#include "dstar_lite.hpp"
#include "dstar_lite/DStarLite.h"

#include <stdexcept>

#include <base/logging.h>

using namespace std;
using namespace Eigen;

namespace nav_graph_search {

DStarLite::DStarLite(const nav_graph_search::TerrainClasses& classes)
{
    mDStarLite = new dstar_lite::DStarLite();

    for(TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); it++)
    {
        mCostMap.insert(std::pair<int,TerrainClass>(it->getNumber(), *it));
        //FIXME add scale
        if(it->getCost() >= 0 && it->getCost() < 1.0) 
        {
            throw std::runtime_error("Error costs between 0 and <1 are not allowed");
        }
    }
    mDStarLite->init(0,0,1,1);
}


DStarLite::~DStarLite() {
    delete mDStarLite;
}

void DStarLite::updateTraversability(int x, int y, int terrain_class)
{
    std::map<int,TerrainClass>::iterator it = mCostMap.find(terrain_class);
    if(it == mCostMap.end()) {
        LOG_WARN("Passed terrain class %d is unknown, traversability map has not been updated!", terrain_class); 
        return;
    }
    
    if(it->second.isTraversable())
    {
        //FIXME scale klass cost
        mDStarLite->updateCell(x + 1, y + 1, it->second.getCost());
    }
    else
    {
        //magic value for not traversable
        mDStarLite->updateCell(x + 1, y + 1, -1.0);
    }
}

void DStarLite::updateTraversabilityMap(envire::TraversabilityGrid* new_grid, envire::TraversabilityGrid* last_grid)
{    
    int updateCnt = 0;
    envire::TraversabilityGrid::ArrayType &new_data(new_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    
    // Sets the DStar-Lite map to new_grid (deletes old data).
    if(!last_grid || (new_grid->getScaleX() != last_grid->getScaleX()) || (new_grid->getScaleY() != last_grid->getScaleY()))
    {
        LOG_INFO("Received new traversability map, a full update is executed");
        mDStarLite->init(mStartPos[0], mStartPos[1], mGoalPos[0], mGoalPos[1]);
	    for(size_t x = 0; x <new_grid->getCellSizeX(); x++)
	    {
	        for(size_t y = 0; y <new_grid->getCellSizeY(); y++)
	        {
                updateTraversability(x, y, new_data[y][x]);
                updateCnt++;
	        }
	    }
    } 
    else
    {
        // Integrates the new grid into DStar-Lite.
        // TODO Actually just 'updateTraversability(x, y, new_data[y][x])' would do the same with the same effort.
        // TODO The pose of the received grid is not regarded, so all grids are mapped onto each other!?
        envire::TraversabilityGrid::ArrayType& last_data(last_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
        for(size_t x = 0; x <new_grid->getCellSizeX(); x++)
        {
            for(size_t y = 0; y <new_grid->getCellSizeY(); y++)
            {
                // Transfers the grid coordinates into the new_grid map.
                Eigen::Vector3d point_new_grid_map = new_grid->fromGrid(x, y); 
                
                size_t x_last_grid, y_last_grid;
                // Transfers the point from the new grid map into the last grid.
                if(last_grid->toGrid(point_new_grid_map, x_last_grid, y_last_grid, new_grid->getFrameNode()))
                {
                    if(new_data[y][x] != last_data[y_last_grid][x_last_grid])
                    {
                        updateTraversability(x, y, new_data[y][x]);
                        updateCnt++;
                    }
                }
                else // Not within the grid.
                {
                    // Old map did not contain the data, so it MUST be new.
                    updateTraversability(x, y, new_data[y][x]);
                    updateCnt++;
                }
            }
        }
    }

    // Draws a non traversable border once or again if the grid size has changed. 
    if(!last_grid || (last_grid->getCellSizeX() != new_grid->getCellSizeX()) || 
            (last_grid->getCellSizeY() != new_grid->getCellSizeY()) || 
            (new_grid->getScaleX() != last_grid->getScaleX()) || 
            (new_grid->getScaleY() != last_grid->getScaleY()))
    {
        LOG_INFO("Adds a non traversable border to the grid");
        for(size_t x = 0; x <new_grid->getCellSizeX() + 2; x++)
        {
            mDStarLite->updateCell(x, 0, -1);
            mDStarLite->updateCell(x, new_grid->getCellSizeY() + 1, -1); // No +2 here!
        }

        for(size_t y = 0; y <new_grid->getCellSizeY() + 2; y++)
        {
            mDStarLite->updateCell(0, y, -1);
            mDStarLite->updateCell(new_grid->getCellSizeX() + 1, y, -1);
        }
    }
}

bool DStarLite::run(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const Eigen::Affine3d &from_world)
{
    const Eigen::Vector3d start_local = from_world * start;
    const Eigen::Vector3d goal_local = from_world * goal;
    return run(goal_local.x(), goal_local.y(), start_local.x(), start_local.y());
}

bool DStarLite::run(int goal_x, int goal_y, int start_x, int start_y) 
{
    std::cout << "Planning from " << start_x << " " << start_y << " to " << goal_x << " " << goal_y << std::endl;
    
    if(goal_x != mGoalPos.x() || goal_y !=  mGoalPos.y()) {
        LOG_INFO("Received new goal position (%d,%d)", goal_x, goal_y);
        // Transform coordinates to allow border.
        mGoalPos = Vector2i(goal_x, goal_y) + Vector2i(1,1);
        mDStarLite->updateGoal(mGoalPos.x(), mGoalPos.y());
    }

    if(start_x != mStartPos.x() || start_y != mStartPos.y()) {
        // Transform coordinates to allow border.
        mStartPos = Vector2i(start_x, start_y) + Vector2i(1,1);
        mDStarLite->updateStart(mStartPos.x(), mStartPos.y());
    }

    if(!mDStarLite->replan()) {
        LOG_WARN("Path could not be found, goal will be reset once");
        mDStarLite->resetGoal();
        if(!mDStarLite->replan()) {
            return false;
        }
    }

    return true;
}

vector< base::geometry::Spline< 3 > > DStarLite::getTrajectory(const Eigen::Affine3d& to_world) const
{
    std::list<dstar_lite::state> path = mDStarLite->getPath();
    std::vector<base::Vector3d> pathWorld;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
        pathWorld.push_back(to_world * Vector3d(it->x - 1, it->y - 1, 0));
    }

    vector< base::geometry::Spline< 3 > > ret;
    base::geometry::Spline< 3 > finalPath;
    finalPath.interpolate(pathWorld);
    ret.push_back(finalPath);
    return ret;
}


std::vector<Eigen::Vector2i> DStarLite::getLocalTrajectory() const
{
    std::list<dstar_lite::state> path = mDStarLite->getPath();
    std::vector<Eigen::Vector2i> ret;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
        ret.push_back(Eigen::Vector2i(it->x - 1, it->y - 1));
    }
    return ret;
}

} // end namespace nav_graph_search


