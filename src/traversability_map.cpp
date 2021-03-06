#include "traversability_map.hpp"

#include <memory>
#include <gdal.h>
#include <gdal_priv.h>
#include <stdexcept>
#include <iomanip>

#include <Eigen/LU>
#include <Eigen/SVD>

using namespace std;
using namespace nav_graph_search;

PointID TraversabilityMap::toLocal(Eigen::Vector3d const& v) const
{
    Eigen::Affine3d world_to_local(getLocalToWorld().inverse());
    Eigen::Vector3i raster3d = (world_to_local * v).cast<int>();
    return PointID(raster3d.x(), raster3d.y());
}

Eigen::Vector3d TraversabilityMap::toWorld(PointID const& v) const
{
    return getLocalToWorld() * v.toEigen();
}

TraversabilityMap* TraversabilityMap::load(std::string const& path, TerrainClasses const& classes)
{
    GDALAllRegister();

    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(path.c_str(), GA_ReadOnly));
    if (!set.get())
        throw std::runtime_error("GDAL cannot open the specified file");

    GDALRasterBand* band = set->GetRasterBand(1);
    if (!band)
        throw std::runtime_error("there is no raster band in this file");

    double transform[6];
    set->GetGeoTransform(transform);

    Eigen::Affine3d local_to_world;
    local_to_world.matrix() <<
        transform[1], transform[2], 0, transform[0],
        transform[4], transform[5], 0, transform[3],
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix3d rotation, scaling;
    local_to_world.computeScalingRotation(&scaling, &rotation);
    if (fabs(fabs(scaling(0, 0) - fabs(scaling(1, 1)))) > 0.001)
        throw std::runtime_error("cannot use maps that have non-square pixels");

    int width  = band->GetXSize();
    int height = band->GetYSize();

    vector<uint8_t> data(width * height);
    band->RasterIO(GF_Read, 0, 0, width, height,
	    &data[0], width, height, GDT_Byte, 0, 0);

    if (!classes.empty())
    {
        int klass_map[256];
        for (int i = 0; i < 256; ++i)
            klass_map[i] = -1;
        for (TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); ++it)
            klass_map[it->in] = it->out;

        for (int i = 0; i < width * height; ++i)
        {
            int out = klass_map[data[i]];
            if (out == -1)
            {
                cerr << "unknown class found: " << static_cast<int>(data[i]) << endl;
                return NULL;
            }
            data[i] = out;
        }
    }

    auto_ptr<TraversabilityMap> map(new TraversabilityMap(width, height,
                local_to_world, 0));
    map->fill(data);
    return map.release();
}

TraversabilityMap* TraversabilityMap::load(envire::Grid<uint8_t> const& map, std::string const& band_name, TerrainClasses const& classes)
{
    envire::Grid<uint8_t>::ArrayType const& input_data = map.getGridData(band_name);
    int xSize = map.getCellSizeX();
    int ySize = map.getCellSizeY();
    envire::Environment& env = *map.getEnvironment();
    // generate the proper local to world transform, which is based
    // on the framenode to root, as well as the offset
    Eigen::Affine3d local_to_world = 
	env.relativeTransform(map.getFrameNode(), env.getRootNode()) * 
	Eigen::Translation3d( map.getOffsetX(), map.getOffsetY(), 0 );

    if(fabs(map.getScaleX() - map.getScaleY()) > 0.00001)
	throw std::runtime_error("Scaling of non square pixels is not supported");

    //set correct scale
    local_to_world.scale(map.getScaleX());
    
    auto_ptr<TraversabilityMap> result(new TraversabilityMap(xSize, ySize,
                local_to_world, 0));

    if (!classes.empty())
    {
        int klass_map[256];
        for (int i = 0; i < 256; ++i)
            klass_map[i] = -1;
        for (TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); ++it)
            klass_map[it->in] = it->out;

        for (int y = 0; y < ySize; ++y)
        {
            uint8_t const* line = &input_data[y][0];
            for (int x = 0; x < xSize; ++x)
            {
                int out = klass_map[line[x]];
                if (out == -1)
                {
                    cerr << "unknown class found at " << x << "x" << y << ": " << static_cast<int>(line[x]) << endl;
                    return NULL;
                }
                result->setValue(x,y, out);
            }
        }
    }

    return result.release();
}

TraversabilityMap::TraversabilityMap(size_t width, size_t height, uint8_t init, float scale)
    : GridMap(width, height)
    , m_values((width * height + 1) / 2, init), m_scale(scale) { }

TraversabilityMap::TraversabilityMap(size_t width, size_t height,
        Eigen::Affine3d const& local_to_world, uint8_t init)
    : GridMap(width, height)
    , m_local_to_world(local_to_world)
    , m_values((width * height + 1) / 2, init)
{
    Eigen::Matrix3d rotation, scaling;
    local_to_world.computeScalingRotation(&scaling, &rotation);
    m_scale = scaling(0, 0);
}

float TraversabilityMap::getScale() const { return fabs(m_scale); }

void TraversabilityMap::fill(vector<uint8_t> const& values)
{
    m_values.resize(values.size() / 2);
    for (int i = 0; i < (int)m_values.size(); ++i)
    {
        m_values[i] = 
            static_cast<int>(values[2 * i]) |
            static_cast<int>(values[2 * i + 1] << 4);
    }
    if (values.size() % 2 == 1)
        m_values.push_back(values[values.size() - 1]);
}
void TraversabilityMap::fill(uint8_t value)
{ 
    value &= 0xF;
    std::fill(m_values.begin(), m_values.end(), (value | value << 4));
}

uint8_t TraversabilityMap::getValue(size_t id) const
{
    int array_idx = id / 2;
    int shift = (id & 1) * 4;
    return (m_values[array_idx] & (0xF << shift)) >> shift;
}
uint8_t TraversabilityMap::getValue(size_t x, size_t y) const
{ return getValue(getCellID(x, y)); }
void TraversabilityMap::setValue(size_t id, uint8_t value)
{
    int array_idx = id / 2;
    int shift = (id & 1) * 4;
    m_values[array_idx] = (m_values[array_idx] & ~(0xF << shift)) | (value << shift);
}
void TraversabilityMap::setValue(size_t x, size_t y, uint8_t value)
{ return setValue(getCellID(x, y), value); }

void TraversabilityMap::writeToPPM(const std::string& filename)
{
    uint8_t max = 0;
    for( int y = 0; y < ySize(); ++y )
    {
	for( int x = 0; x < xSize(); ++x )
	{
	    max = std::max(max, getValue(x, y));
	}
    }
    
    std::cout << "Tr map max is " << (int) max << std::endl;
    
    std::ofstream cost_img;
    cost_img.open((filename + std::string(".ppm")).c_str());
    cost_img << "P6" << std::endl;
    cost_img << xSize() << " " << ySize() << std::endl;
    cost_img << "255" << std::endl;
    for( int y = 0; y < ySize(); ++y )
    {
	for( int x = 0; x < xSize(); ++x )
	{
	    uint32_t curVal = getValue( x, y);
	    uint8_t val = max == 0 ? 255 : curVal * 255 / max;
	    cost_img.write((const char *) &val, 1);
	    cost_img.write((const char *) &val, 1);
	    cost_img.write((const char *) &val, 1);
	}
    }    
    cost_img.close();
}

void TraversabilityMap::dumpToStdOut()
{
    std::cout << std::endl;
    for(int x = 0;x < xSize(); x++)
    {
	std::cout << std::setw(3) << x << " ";
    }
    std::cout << std::endl;
    for(int y = 0; y < ySize(); y++)
    {
	std::cout << std::setw(3) << y << " ";
	for(int x = 0;x < xSize(); x++)
	{
	    std::cout << std::setw(3) << (int) getValue(x, y) << " ";
	}
	std::cout << std::endl;
    }
}


