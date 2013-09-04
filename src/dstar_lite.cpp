#include "dstar_lite.hpp"
#include "dstar_lite/DStarLite.h"

#include <stdexcept>

#include <base/logging.h>

namespace nav_graph_search {

DStarLite::DStarLite(const nav_graph_search::TerrainClasses& classes) : mCostMap(), 
        mDStarLite(NULL), mTravGrid(NULL), mEnv(), mStatistics()
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
    mDStarLite = NULL;
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
        mDStarLite->updateCell(x, y, it->second.getCost());
    }
    else
    {
        // Magic value for not traversable.
        mDStarLite->updateCell(x, y, -1.0);
    }
    mStatistics.mCellsUpdated++;
}

void DStarLite::updateTraversabilityMap(envire::TraversabilityGrid* new_grid)
{   
    if(!new_grid) {
        LOG_WARN("Received NULL pointer instead of a traversability map");
        return;
    }

    envire::TraversabilityGrid::ArrayType &new_data(new_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));

    // First received trav grid is used as the world grid.
    if(!mTravGrid) {

        // Adds a copy of the received trav map and its node to mEnv.
        // An environment is required, because getFrameNode() only works within an env.
        mTravGrid = new envire::TraversabilityGrid(*new_grid);
        mTravGrid->setUniqueId("trav_root_map");
        mEnv.attachItem(mTravGrid);
        envire::FrameNode* p_fn = new envire::FrameNode(*new_grid->getFrameNode());
        mEnv.getRootNode()->addChild(p_fn);
        mTravGrid->setFrameNode(p_fn);
        
        // Update DStar-Lite map.
        LOG_INFO("Received first traversability map, a full update is executed");
        for(size_t x = 0; x <new_grid->getCellSizeX(); x++)
	    {
	        for(size_t y = 0; y <new_grid->getCellSizeY(); y++)
	        {
                updateTraversability(x, y, new_data[y][x]);
	        }
	    }
	    
	    // Draw border.
	    LOG_INFO("Adds a non traversable border around the grid");
        for(size_t x = -1; x <new_grid->getCellSizeX() + 1; x++)
        {
            mDStarLite->updateCell(x, -1, -1);
            mDStarLite->updateCell(x, new_grid->getCellSizeY(), -1);
            mStatistics.mCellsUpdated += 2;
        }

        for(size_t y = -1; y <new_grid->getCellSizeY() + 1; y++)
        {
            mDStarLite->updateCell(-1, y, -1);
            mDStarLite->updateCell(new_grid->getCellSizeX(), y, -1);
            mStatistics.mCellsUpdated += 2;
        }
    } 
    else // Adds the received grid to the existing one. TODO Test this!
    {
        envire::FrameNode* fn_root = mTravGrid->getFrameNode();
        
        // Test start
        /*
        envire::FrameNode* new_fn = new_grid->getFrameNode();
        envire::Transform trans = new_fn->getTransform();
        int x = rand()%36;
        int y = rand()%28;
        std::cout << "New grid position: " << x << ", " << y << std::endl; 
        trans.translate(base::Vector3d(x,y,0));
        new_fn->setTransform(trans);
        new_grid->setFrameNode(new_fn);
        */
        // Test end
        
        // The map has to be copied because it is not possible to contain the same
        // map within two environments. And it has to be placed within the environment
        // to transform between this and the root map.
        envire::TraversabilityGrid* new_grid_copy = new envire::TraversabilityGrid(*new_grid);
        mEnv.attachItem(new_grid_copy);
        envire::FrameNode* p_fn = new envire::FrameNode(*new_grid->getFrameNode());
        mEnv.getRootNode()->addChild(p_fn);
        new_grid_copy->setFrameNode(p_fn);
        
        size_t x_root = 0, y_root = 0;
        int new_class = 0;
        double cost = 0, new_cost = 0;
        int not_within_root_map_counter = 0;
        int update_cells = 0;
        for(size_t x_new = 0; x_new <new_grid_copy->getCellSizeX(); x_new++)
	    {
	        for(size_t y_new = 0; y_new <new_grid_copy->getCellSizeY(); y_new++)
	        {
	            // Transfers the coordinate from the new grid to the root grid. 
	            base::Vector3d p_root_map = new_grid_copy->fromGrid(x_new, y_new, fn_root);
	            bool within_grid = mTravGrid->toGrid(p_root_map.x(), p_root_map.y(), x_root, y_root);
	            //LOG_DEBUG("The position of the new grid coordinate (%d,%d) within the root frame is (%4.2f, %4.2f, %4.2f) and within the root grid is (%d,%d)", 
	            //        x_new, y_new, p_root_map[0], p_root_map[1], p_root_map[2], x_root, y_root );
	            if(!within_grid) {
	                //LOG_DEBUG("New grid coordinate (%d,%d) does not lie within the root grid", x_new, y_new);
                    ++not_within_root_map_counter;
	                continue;
	            }
	            // Request the cost within the DStar-Lite map.
	            if(getCost(x_root, y_root, cost)) { // Cell available.
	                new_class = new_data[y_new][x_new];
   	                // Convert class to cost.
	                std::map<int,TerrainClass>::iterator it = mCostMap.find(new_class);
	                if(it == mCostMap.end()) {
	                    LOG_WARN("Received unknown terrain class %d", new_class);
	                    continue;
	                }
	                new_cost = it->second.cost;
	                if(cost != new_cost) {
                        updateTraversability(x_new, y_new, new_class);
                        ++update_cells;
                    }
                } else { // Cell has not been added yet.
                    updateTraversability(x_new, y_new, new_class);
                    ++update_cells;
                }
	        }
	    }
	    double num_cells = new_grid_copy->getCellSizeX() * new_grid_copy->getCellSizeY();
	    LOG_INFO("%d (%4.2f \%) cells of the new grid have been positioned outside of the root grid", 
	            not_within_root_map_counter, 100 * not_within_root_map_counter / num_cells);
	    LOG_INFO("%d (%4.2f \%) have been updated", update_cells, 100 * update_cells / num_cells);
    }
}

bool DStarLite::run(const base::Vector3d& start_map, const base::Vector3d& goal_map)
{
    size_t start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
    mTravGrid->toGrid(start_map.x(), start_map.y(), start_x, start_y);
    mTravGrid->toGrid(goal_map.x(), goal_map.y(), goal_x, goal_y);
    return run(goal_x, goal_y, start_x, start_y);
}

bool DStarLite::run(int goal_x, int goal_y, int start_x, int start_y) 
{
    LOG_INFO("Planning from (%d,%d) to (%d,%d)", start_x, start_y, goal_x, goal_y);
    
    if(goal_x != mGoalPos.x() || goal_y !=  mGoalPos.y()) {
        LOG_INFO("Received new goal position (%d,%d)", goal_x, goal_y);
        mGoalPos = Eigen::Vector2i(goal_x, goal_y);
        mDStarLite->updateGoal(mGoalPos.x(), mGoalPos.y());
    }

    if(start_x != mStartPos.x() || start_y != mStartPos.y()) {
        mStartPos = Eigen::Vector2i(start_x, start_y);
        mDStarLite->updateStart(mStartPos.x(), mStartPos.y());
    }

    if(!mDStarLite->replan()) {
        LOG_WARN("Path could not be found, goal will be reset once");
        mStatistics.mFailedPlanning++; 
        mStatistics.mOverallPlanning++;
        mDStarLite->resetGoal();
        if(!mDStarLite->replan()) {
            mStatistics.mFailedPlanning++; 
            mStatistics.mOverallPlanning++;
            return false;
        }
    }
    mStatistics.mOverallPlanning++;
    return true;
}

std::vector< base::geometry::Spline< 3 > > DStarLite::getSplineMap() const
{
    std::list<dstar_lite::state> path = mDStarLite->getPath();
    std::vector<base::Vector3d> pathWorld;
    double x_world = 0.0, y_world = 0.0;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
        mTravGrid->fromGrid(it->x, it->y, x_world, y_world);
        pathWorld.push_back(base::Vector3d(x_world, y_world, 0));
    }

    std::vector< base::geometry::Spline< 3 > > ret;
    base::geometry::Spline< 3 > finalPath;
    finalPath.interpolate(pathWorld);
    ret.push_back(finalPath);
    return ret;
}

std::vector<Eigen::Vector2i> DStarLite::getTrajectoryGrid() const
{
    std::list<dstar_lite::state> path = mDStarLite->getPath();
    std::vector<Eigen::Vector2i> ret;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
        ret.push_back(Eigen::Vector2i(it->x, it->y));
    }
    return ret;
}

std::vector<base::Vector3d> DStarLite::getTrajectoryMap() const
{
    std::list<dstar_lite::state> path = mDStarLite->getPath();
    std::vector<base::Vector3d> ret;
    for(std::list<dstar_lite::state>::iterator it = path.begin(); it != path.end(); it++)
    {
        ret.push_back(mTravGrid->fromGrid(it->x, it->y));
    }
    return ret;
}

bool DStarLite::getCost(int x, int y, double& cost) {
    double cost_tmp;
    if(mDStarLite->getCost(x, y, cost_tmp)) {
        cost = cost_tmp;
        return true;
    } else { // Cell not available yet.
        return false;
    }
}

} // end namespace nav_graph_search


