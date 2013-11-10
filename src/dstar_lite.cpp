#include "dstar_lite.hpp"
#include "dstar_lite/DStarLite.h"

#include <stdexcept>

#include <base/logging.h>

namespace nav_graph_search {

DStarLite::DStarLite(const nav_graph_search::TerrainClasses& classes) : mClass2CostMap(), 
        mCost2ClassMap(), mDStarLite(NULL), mTravGrid(NULL), mEnv(), mStatistics(),
        mNewTravGrid(NULL), mNewTravFrameNode(NULL), mRemoveObstaclesRadius(0.0)
{
    mDStarLite = new dstar_lite::DStarLite();

    for(TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); it++)
    {
        mClass2CostMap.insert(std::pair<int,TerrainClass>(it->getNumber(), *it));
        mCost2ClassMap.insert(std::pair<float,int>(it->getCost(), it->getNumber()));
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
    std::map<int,TerrainClass>::iterator it = mClass2CostMap.find(terrain_class);
    if(it == mClass2CostMap.end()) {
        LOG_WARN("DStarLite: Passed terrain class %d is unknown, traversability map has not been updated!", terrain_class); 
        return;
    }
    
    double new_cost = -1; // Magic value for not traversable.
    if(it->second.isTraversable()) {
        new_cost = it->second.getCost(); // FIXME scale klass cost
    }
    mDStarLite->updateCell(x, y, new_cost);
    
    // There seem to be a prpoblem during the first complete map update 
    // (before the borders have been added) to update cell (0,0) and (1,1)
    double current_cost = 0.0;
    mDStarLite->getCost(x, y, current_cost);
    if(it->second.getCost() != current_cost) {
        LOG_WARN("DStarLite: The current cost %4.2f of cell (%d,%d) could not been updated with %4.2f", 
                current_cost, x, y, new_cost);
    }
    
    mStatistics.mCellsUpdated++;
}

void DStarLite::updateTraversabilityMap(envire::TraversabilityGrid* new_grid)
{   
    if(!new_grid) {
        LOG_WARN("DStarLite: Received NULL pointer instead of a traversability map");
        return;
    }

    envire::TraversabilityGrid::ArrayType &new_data(new_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));

    // First received trav grid is used as the world grid.
    if(mTravGrid != NULL) {
        mEnv.detachItem(mTravGrid->getFrameNode());
        mEnv.detachItem(mTravGrid);
    }

    // Adds a copy of the received trav map and its node to mEnv.
    // An environment is required, because getFrameNode() only works within an env.
    mTravGrid = new envire::TraversabilityGrid(*new_grid);
    mTravGrid->setUniqueId("root_trav_map");
    mEnv.attachItem(mTravGrid);
    envire::FrameNode* p_fn = new envire::FrameNode(*new_grid->getFrameNode());
    p_fn->setUniqueId("root_frame_node");
    mEnv.getRootNode()->addChild(p_fn);
    mTravGrid->setFrameNode(p_fn);
    
    // Update DStar-Lite map.
    LOG_INFO("DStarLite: Received first traversability map, a full update is executed");
    for(size_t x = 0; x <new_grid->getCellSizeX(); x++)
    {
        for(size_t y = 0; y <new_grid->getCellSizeY(); y++)
        {
            updateTraversability(x, y, new_data[y][x]);
        }
    }
    
    
//     else // Adds the received grid to the existing one.
//     {
//         envire::FrameNode* fn_root = mTravGrid->getFrameNode();
//         
//         // Just for testing, translate new received map
//         /*  
//         envire::FrameNode* new_fn = new_grid->getFrameNode();
//         envire::Transform trans = new_fn->getTransform();
//         int x = -18;//rand()%36;
//         int y = -14;//rand()%28;
//         std::cout << "New grid position: " << x << ", " << y << std::endl; 
//         trans.translate(base::Vector3d(x,y,0));
//         new_fn->setTransform(trans);    
//         */
//         // Test end
//         
//         // Attach the map to the environment to calculate the transformation. 
//         if(mNewTravGrid != NULL) {
//             mEnv.detachItem(mNewTravGrid);
//         }
//         //mNewTravGrid->operator=( *new_grid ); // vorher 
//         
//         mNewTravGrid = new envire::TraversabilityGrid(*new_grid);
//         mNewTravGrid->setUniqueId("new_trav_map");
//         mEnv.attachItem(mNewTravGrid);
//         
//         if(mNewTravFrameNode == NULL) {
//             mNewTravFrameNode = new envire::FrameNode(*new_grid->getFrameNode());
//             mNewTravFrameNode->setUniqueId("new_frame_node");
//             mEnv.getRootNode()->addChild(mNewTravFrameNode);
//         }
//         mNewTravGrid->setFrameNode(mNewTravFrameNode);   
//         
//         size_t x_root = 0, y_root = 0;
//         int new_class = 0;
//         double cost = 0, new_cost = 0;
//         int not_within_root_map_counter = 0;
//         int update_cells = 0;
//         for(size_t x_new = 0; x_new < mNewTravGrid->getCellSizeX(); x_new++)
// 	    {
// 	        for(size_t y_new = 0; y_new < mNewTravGrid->getCellSizeY(); y_new++)
// 	        {
// 	            // Transfers the coordinate from the new grid to the root grid. 
// 	            base::Vector3d p_root_map = mNewTravGrid->fromGrid(x_new, y_new, fn_root);
// 	            bool within_grid = mTravGrid->toGrid(p_root_map.x(), p_root_map.y(), x_root, y_root);
// 	            if(!within_grid) {
// 	                //LOG_DEBUG("New grid coordinate (%d,%d) does not lie within the root grid", x_new, y_new);
//                     ++not_within_root_map_counter;
// 	                continue;
// 	            }
// 	            // Request the cost within the DStar-Lite map.
// 	            if(getCost(x_root, y_root, cost)) { // Cell available.
// 	                new_class = new_data[y_new][x_new];
//    	                // Convert terrain class to cost.
// 	                std::map<int,TerrainClass>::iterator it = mClass2CostMap.find(new_class);
// 	                if(it == mClass2CostMap.end()) {
// 	                    LOG_WARN("Received unknown terrain class %d", new_class);
// 	                    continue;
// 	                }
// 	                new_cost = it->second.cost;
// 	                if(cost != new_cost) {
//                         updateTraversability(x_root, y_root, new_class);
//                         ++update_cells;
//                     }
//                 } else { // Cell has not been added yet.
//                     updateTraversability(x_root, y_root, new_class);
//                     ++update_cells;
//                 }
// 	        }
// 	    }
// 	    double num_cells = mNewTravGrid->getCellSizeX() * mNewTravGrid->getCellSizeY();
// 	    LOG_INFO("%d (%4.2f \%) cells of the new grid have been positioned outside of the root grid", 
// 	            not_within_root_map_counter, 100 * not_within_root_map_counter / num_cells);
// 	    LOG_INFO("%d (%4.2f \%) have been updated", update_cells, 100 * update_cells / num_cells);
//     }
    
    // Draw border.
    LOG_INFO("DStarLite: Adds a non traversable border around the grid");
    for(size_t x = -1; x <mTravGrid->getCellSizeX() + 1; x++)
    {
        mDStarLite->updateCell(x, -1, -1);
        mDStarLite->updateCell(x, mTravGrid->getCellSizeY(), -1);
        mStatistics.mCellsUpdated += 2;
    }

    for(size_t y = -1; y <mTravGrid->getCellSizeY() + 1; y++)
    {
        mDStarLite->updateCell(-1, y, -1);
        mDStarLite->updateCell(mTravGrid->getCellSizeX(), y, -1);
        mStatistics.mCellsUpdated += 2;
    }
    
}

bool DStarLite::run(const base::Vector3d& start_map, const base::Vector3d& goal_map, enum Error* error)
{
    base::Vector3d start_world = start_map;
    base::Vector3d goal_world = goal_map;
    
    start_world.z() = 0;
    goal_world.z() = 0;
    
    size_t start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
    if(!mTravGrid->toGrid(start_world, start_x, start_y, mTravGrid->getEnvironment()->getRootNode()))
    {
        LOG_ERROR("DStarLite: Error start pos is out of grid");
        return false;
    }
    if(!mTravGrid->toGrid(goal_world, goal_x, goal_y, mTravGrid->->getEnvironment()->getRootNode()))
    {
        LOG_ERROR("DStarLite: Error goal pos is out of grid");
        return false;
    }
    
    return run(goal_x, goal_y, start_x, start_y, error);
}

bool DStarLite::run(int goal_x, int goal_y, int start_x, int start_y, enum Error* error) 
{
    LOG_INFO("DStarLite: Planning from (%d,%d) to (%d,%d)", start_x, start_y, goal_x, goal_y);
    if(error != NULL) {
        *error = NONE;
    }
    
    // If the goal positionis placed on an obstacle, 'error' will be set and false will be returned.
    if(goal_x != mGoalPos.x() || goal_y !=  mGoalPos.y()) {
        LOG_INFO("DStarLite: Received new goal position (%d,%d)", goal_x, goal_y);
        double cost = 0.0;
        if(mDStarLite->getCost(goal_x, goal_y, cost)) { // The cost could be requested (cell is available).
            if(cost < 0) { // Obstacle.
                if(error != NULL) {
                    *error = GOAL_SET_ON_OBSTACLE;
                }
                LOG_WARN("DStarLite: New goal position has been placed on an obstacle, no planning will be exceuted");
                return false;
            }
        }
        
        mGoalPos = Eigen::Vector2i(goal_x, goal_y);
        mDStarLite->updateGoal(mGoalPos.x(), mGoalPos.y());
    }

    // If the start position is placed on an obstacle, the obstacle will be removed.
    if(start_x != mStartPos.x() || start_y != mStartPos.y()) {
        double cost = 0.0;
        if(mDStarLite->getCost(start_x, start_y, cost)) { // The cost could be requested (cell is available).
            if(cost < 0) { // Obstacle.
                LOG_WARN("DStarLite: The start position (%d,%d) has been placed on an obstacle. The obstacle will be removed.");
            }
        }
    
        mStartPos = Eigen::Vector2i(start_x, start_y);
        mDStarLite->updateStart(mStartPos.x(), mStartPos.y());
    }

    // Removes obstacles around the robot.
    if(mRemoveObstaclesRadius > 0 && mTravGrid  != NULL) {
        // Convert radius to grid cells, assuming scaleX == scaleY.
        int radius_cells = mRemoveObstaclesRadius * mTravGrid->getScaleX();
        int left_x = start_x - radius_cells, right_x = start_x + radius_cells;
        int top_y = start_y - radius_cells, bottom_y = start_y + start_y;
        if(left_x < 0)
            left_x = 0;
        if(right_x > (int)mTravGrid->getCellSizeX() - 1)
            right_x = (int)mTravGrid->getCellSizeX() - 1;
        if(top_y < 0)
            top_y = 0;
        if(bottom_y > (int)mTravGrid->getCellSizeY() - 1)
            bottom_y = (int)mTravGrid->getCellSizeY() - 1;

        // Remove all obstacles within the choosen radius.
        int distance_cells = 0;
        double cost = 0.0;
        int counter_removed_obstacles = 0;
        int counter_patches_no_cost = 0;
        int counter_patches = 0;
        bool ret;
        for(int x = left_x; x < right_x; ++x) {
            for(int y = top_y; y < bottom_y; ++y) {
                distance_cells = sqrt((start_x - x) * (start_x - x) + (start_y - y) * (start_y - y)); 
                if(distance_cells < radius_cells) {
                    counter_patches++;
                    cost = -1.0;
                    ret = mDStarLite->getCost(x, y, cost);
                    if(!ret) { // Patches not yet available.
                        counter_patches_no_cost++;
                    }
                    if(ret && cost == -1) { // Obstacle found.
                        counter_removed_obstacles++;
                    }
                    if(cost < 0) { // Obstacle or not yet available.
                        updateTraversability(x,y,12); // WARN Using fix class value here.
                    }
                }                   
            }
        } 
        LOG_INFO("DStarLite: %d patches within %4.2f m (%d grids) of the robot position (%d, %d)", counter_patches, mRemoveObstaclesRadius, radius_cells, start_x, start_y);
        LOG_INFO("DStarLite: %d obstacles removed, %d new patches created", counter_removed_obstacles, counter_patches_no_cost);
    }

    if(!mDStarLite->replan()) {
        LOG_WARN("DStarLite: Path could not be found, goal will be reset once");
        mStatistics.mFailedPlanning++; 
        mStatistics.mOverallPlanning++;
        mDStarLite->resetGoal();
        if(!mDStarLite->replan()) {
            mStatistics.mFailedPlanning++; 
            mStatistics.mOverallPlanning++;
            if(error != NULL) {
                *error = NO_PATH_TO_GOAL;
            }
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
        if(!mTravGrid->contains(envire::GridBase::Position(x_world, y_world)))
            throw std::runtime_error("Got path that is out of map");

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
        ret.push_back(mTravGrid->fromGrid(it->x, it->y, mTravGrid->getEnvironment()->getRootNode()));
    }
    return ret;
}

bool DStarLite::getCost(int x, int y, double& cost) {
    double cost_tmp = 0.0;
    if(mDStarLite->getCost(x, y, cost_tmp)) {
        cost = cost_tmp;
        return true;
    } else { // Cell not available yet.
        return false;
    }
}

bool DStarLite::getCostWorld(double x, double y, double& cost)
{
    size_t x_local;
    size_t y_local;
    Eigen::Vector3d pos(x, y, 0);
    if(!mTravGrid->toGrid(pos, x_local, y_local, mTravGrid->getEnvironment()->getRootNode()))
    {
        return false;
    }
    return getCost(x_local, y_local, cost);
}


bool DStarLite::getTerrainClass(int x, int y, int& class_) {
    double cost_tmp = 0.0;
    if(mDStarLite->getCost(x, y, cost_tmp)) {
        std::map<float,int>::iterator it = mCost2ClassMap.find(cost_tmp);
        if(it == mCost2ClassMap.end()) {
            LOG_WARN("DStarLite: Mapping of cost %4.2f to terrain-class is not available", cost_tmp);
            return false;
        } else {
            class_ = it->second;
            return true;
        }
    } else { // Cell not available yet.
        return false;
    }
}

} // end namespace nav_graph_search


