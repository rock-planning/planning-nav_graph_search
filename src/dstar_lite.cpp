#include "dstar_lite.hpp"
#include "dstar_lite/DStarLite.h"

#include <stdexcept>

#include <base/logging.h>

using namespace std;
using namespace Eigen;

namespace nav_graph_search {

DStarLite::DStarLite(const nav_graph_search::TerrainClasses& classes)
{
    m_dstarLite = new dstar_lite::DStarLite();
    costMap.resize(std::numeric_limits< uint8_t >::max());
    for(TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); it++)
    {
	costMap[it->getNumber()] = *it;
	//FIXME add scale
	if(it->getCost() >= 0 && it->getCost() < 1.0)
	    throw std::runtime_error("Error costs between 0 and <1 are not allowed");
    }
    m_dstarLite->init(0,0,1,1);
    
}


DStarLite::~DStarLite() {
    delete m_dstarLite;
}

void DStarLite::updateTraversability(int x, int y, int klass)
{
    const TerrainClass &curKlass(costMap[klass]);
    if(curKlass.isTraversable())
    {
	//FIXME scale klass cost
	m_dstarLite->updateCell(x + 1, y + 1, curKlass.getCost());
    }
    else
    {
	//magic value for not traversable
	m_dstarLite->updateCell(x + 1, y + 1, -1.0);
    }
}

void DStarLite::updateTraversabilityMap(envire::TraversabilityGrid* newGrid, envire::TraversabilityGrid* curGrid)
{    
    std::cout << "Got Grid" << std::endl;
    int updateCnt = 0;
    envire::FrameNode *newGridFrame = newGrid->getFrameNode();
    envire::TraversabilityGrid::ArrayType &newData(newGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));

    //do full update if scale changed
    if(!curGrid || (newGrid->getScaleX() != curGrid->getScaleX()) || (newGrid->getScaleY() != curGrid->getScaleY()))
    {
        m_dstarLite->init(0,0,1,1);
        
	std::cout << "No old grid taking new one " << std::endl;
	for(size_t x = 0; x <newGrid->getCellSizeX(); x++)
	{
	    for(size_t y = 0; y <newGrid->getCellSizeY(); y++)
	    {
		//old map did not contain the data. so it MUST be new
		updateTraversability(x, y, newData[y][x]);
                updateCnt++;
	    }
	}
    } 
    else
    {

        envire::FrameNode *oldGridFrame = newGrid->getFrameNode();
        envire::TraversabilityGrid::ArrayType &oldData(curGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
        for(size_t x = 0; x <newGrid->getCellSizeX(); x++)
        {
            for(size_t y = 0; y <newGrid->getCellSizeY(); y++)
            {
                Eigen::Vector3d posWorld = newGrid->fromGrid(x, y, newGridFrame);
                
                size_t xOldGrid, yOldGrid;
                
                if(curGrid->toGrid(posWorld, xOldGrid, yOldGrid, oldGridFrame))
                {
                    if(newData[y][x] != oldData[yOldGrid][xOldGrid])
                    {
                        updateTraversability(x, y, newData[y][x]);
                        updateCnt++;
                    }
                }
                else
                {
                    //old map did not contain the data. so it MUST be new
                    updateTraversability(x, y, newData[y][x]);
                    updateCnt++;
                }
            }
        }
    }

    if(!curGrid || (curGrid->getCellSizeX() != newGrid->getCellSizeX()) || (curGrid->getCellSizeY() != newGrid->getCellSizeY())
       || (newGrid->getScaleX() != curGrid->getScaleX()) || (newGrid->getScaleY() != curGrid->getScaleY()))
    std::cout << "Updating Grid" << std::endl;
    {
        //add non traversable border
        for(size_t x = 0; x <newGrid->getCellSizeX() + 2; x++)
        {
            m_dstarLite->updateCell(x, 0, -1);
            m_dstarLite->updateCell(x, newGrid->getCellSizeY() + 2, -1);
        }

        for(size_t y = 0; y <newGrid->getCellSizeY() + 2; y++)
        {
            m_dstarLite->updateCell(0, y, -1);
            m_dstarLite->updateCell(newGrid->getCellSizeX() + 2, y, -1);
        }
    }
}

bool DStarLite::run(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const Eigen::Affine3d &fromWorld)
{
    const Eigen::Vector3d startLocal = fromWorld * start;
    const Eigen::Vector3d goalLocal = fromWorld * goal;
    return run(goalLocal.x(), goalLocal.y(), startLocal.x(), startLocal.y());
}




bool DStarLite::run(int goal_x, int goal_y, int start_x, int start_y) 
{
    std::cout << "Planning from " << start_x << " " << start_y << " to " << goal_x << " " << goal_y << std::endl;
    
    if(goal_x != goal.x() || goal_y !=  goal.y()) {
        LOG_INFO("Received new goal position (%d,%d)", goal_x, goal_y);
	//transform coordinates to allow border
	goal = Vector2i(goal_x, goal_y) + Vector2i(1,1);
        m_dstarLite->updateGoal(goal.x(), goal.y());
    }

    if(start_x != start.x() || start_y != start.y()) {
	//transform coordinates to allow border
	start = Vector2i(start_x, start_y) + Vector2i(1,1);
        m_dstarLite->updateStart(start.x(), start.y());
    }

    if(!m_dstarLite->replan()) {
        LOG_WARN("Path could not be found, goal will be reset once");
        m_dstarLite->resetGoal();
        if(!m_dstarLite->replan()) {
	    return false;
        }
    }

    return true;
}

vector< base::geometry::Spline< 3 > > DStarLite::getTrajectory(const Eigen::Affine3d& toWorld) const
{
    std::list<dstar_lite::state> path = m_dstarLite->getPath();
    std::vector<base::Vector3d> pathWorld;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
	pathWorld.push_back(toWorld * Vector3d(it->x - 1, it->y - 1, 0));
    }
    
    vector< base::geometry::Spline< 3 > > ret;
    base::geometry::Spline< 3 > finalPath;
    finalPath.interpolate(pathWorld);
    ret.push_back(finalPath);
    return ret;
}


std::vector<Eigen::Vector2i> DStarLite::getLocalTrajectory() const
{
    std::list<dstar_lite::state> path = m_dstarLite->getPath();
    std::vector<Eigen::Vector2i> ret;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
	ret.push_back(Eigen::Vector2i(it->x - 1, it->y - 1));
    }
    return ret;
}

} // end namespace nav_graph_search


