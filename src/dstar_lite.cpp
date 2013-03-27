#include "dstar_lite.hpp"
#include "dstar_lite/DStarLite.h"

#include <stdexcept>

#include <base/logging.h>

using namespace std;
using namespace Eigen;

namespace nav_graph_search {

DStarLite::DStarLite(const nav_graph_search::TerrainClasses& classes) : needsInit(true), curGrid(NULL)
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
	//FIXME scale klass cost
	m_dstarLite->updateCell(x, y, curKlass.getCost());
    else
	//magic value for not traversable
	m_dstarLite->updateCell(x, y, -1.0);
}

void DStarLite::updateTraversabilityMap(envire::TraversabilityGrid* newGrid)
{
    std::cout << "Got Grid" << std::endl;
    envire::TraversabilityGrid::ArrayType &newData(newGrid->getGridData());
    if(!curGrid)
    {
	std::cout << "No old grid taking new one " << std::endl;
	for(size_t x = 0; x <newGrid->getCellSizeX(); x++)
	{
	    for(size_t y = 0; y <newGrid->getCellSizeY(); y++)
	    {
		//old map did not contain the data. so it MUST be new
		updateTraversability(x, y, newData[y][x]);
	    }
	}
	curGrid = newGrid;
	return;
    }

    std::cout << "Updating Grid" << std::endl;

    envire::TraversabilityGrid::ArrayType &oldData(curGrid->getGridData());
    std::cout << "Size is " << newGrid->getCellSizeX() << " " << newGrid->getCellSizeY() << std::endl;
    for(size_t x = 0; x <newGrid->getCellSizeX(); x++)
    {
	for(size_t y = 0; y <newGrid->getCellSizeY(); y++)
	{
	    Eigen::Vector3d posWorld = newGrid->fromGrid(x, y);
	    
	    size_t xOldGrid, yOldGrid;
	    
	    if(curGrid->toGrid(posWorld, xOldGrid, yOldGrid))
	    {
		if(newData[x][y] != oldData[xOldGrid][yOldGrid])
		{
		    updateTraversability(x, y, newData[y][x]);
		}
	    }
	    else
	    {
		//old map did not contain the data. so it MUST be new
		updateTraversability(x, y, newData[y][x]);
	    }
	    
	}
    }
    delete curGrid;
    curGrid = newGrid;

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
        m_dstarLite->updateGoal(goal_x, goal_y);
	goal = Vector2i(goal_x, goal_y);
    }

    if(start_x != start.x() || start_y != start.y()) {
        m_dstarLite->updateStart(start_x, start_y);
	start = Vector2i(start_x, start_y);
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
	pathWorld.push_back(toWorld * Vector3d(it->x, it->y, 0));
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
	ret.push_back(Eigen::Vector2i(it->x, it->y));
    }
    return ret;
}

} // end namespace nav_graph_search


