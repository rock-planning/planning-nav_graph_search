/*
 * \file    dstar_lite.hpp
 *  
 * \brief   DStar-Lite path planner implementation.
 *
 * \details Based on the first version of http://code.google.com/p/dstarlite/.
 *          
 *          German Research Center for Artificial Intelligence\n
 *          Project: SpaceBot
 *
 * \date    06.03.13
 *
 * \author  Stefan.Haase@dfki.de
 */

#ifndef NAV_DSTAR_LITE_HPP
#define NAV_DSTAR_LITE_HPP

#include <envire/maps/TraversabilityGrid.hpp>
#include <base/geometry/Spline.hpp>

namespace dstar_lite {
class DStarLite;
}

namespace nav_graph_search {

struct Statistics {
 public:
    unsigned int mOverallPlanning;
    unsigned int mFailedPlanning; 
    unsigned int mCellsUpdated;
    
    Statistics() : mOverallPlanning(0), mFailedPlanning(0), mCellsUpdated(0) {}
    
    void reset() {
        mOverallPlanning = 0;
        mFailedPlanning = 0; 
        mCellsUpdated = 0; 
    }
    
    std::string toString() {
        std::stringstream ss;
        ss << "DStar-Lite statistics" << std::endl;
        ss << "Overall number of plannings: " << mOverallPlanning << std::endl;
        ss << "Number of plannings failed: " << mFailedPlanning << std::endl;
        ss << "Number of updated cells: " << mCellsUpdated << std::endl;
        return ss.str();
    }
};

/** 
 * Using the sources within src/dstar_lite and the class TraversabilitySearch
 * to realize DStar-Lite path planning. TraversabilitySearch is just used to 
 * generating and updating the cost map regarding the footprint-size. 
 * The cost values are used (which requires smaller modifications)
 * to build up the internal DStar-Lite cost map.\n
 * In the current version the cost values should be close to 0 (nav_graph cost map)
 * respectively close to 1 (dstar_lite cost map), otherwise the heuristic becomes useless.
 * This can be achieved by using a smaller grid size or different speed units e.g. m/minute.
 * TODO Add statistics like 'number of replans' or 'number of failed replans'.
 */
class DStarLite
{
 public:
    enum Error {NONE, 
            GOAL_ON_OBSTACLE, 
            NO_PATH_TO_GOAL,
            GOAL_OUT_OF_GRID,
            START_OUT_OF_GRID};
 
    /**
     * Creates the cost map by passing the parameters to TraversabilitySearch and
     * allocates the internal dstar_lite::DStarLite object.
     */
    DStarLite();

    /**
     * Deallocates the internal dstar_lite::DStarLite object.
     */
    ~DStarLite();

    /**
     * Convenience function. 
     * Transforms start and goal from map to grid using the transformations
     * from the first received traversability grid.
     */
    bool run(const base::Vector3d &start_map, const base::Vector3d &goal_map, enum Error* error=NULL);
    
    /**
     * Run the algorithm for the given goal and start point.
     * Points have to lie within the world grid.
     * If the goal pos is placed on an obstacle or the planning failed twice (after the first 
     * failed planning the goal position is reset once), false is returned and 'error' will be
     * set to indicate the error. If the start position is placed on an obstacle, the obstacle
     * will be removed.
     */
    bool run(int goal_x, int goal_y, int start_x, int start_y, enum Error* error=NULL);

    /**
     * Updates the Dstar with the given map.
     * */
    void updateTraversabilityMap(envire::TraversabilityGrid* newGrid);
    
    /** 
     * Updates the traversability class to \c klass for the given cell.
     */
    void updateTraversability(int x, int y, int terrain_class);

    /**
     * Returns the spline in map coordinates.
     */
    std::vector<base::geometry::Spline<3> > getSplineMap() const;
    
    /**
     * Returns the path in grid coordinates.
     */
    std::vector<Eigen::Vector2i> getTrajectoryGrid() const;
    
    /**
     * Returns the path in map coordinates.
     */
    std::vector<base::Vector3d> getTrajectoryMap() const;
    
    /**
     * Allows to request the cost of the cell within the DStar-Lite map (world/root frame).
     * If the cell (x,y) has not been added yet false will be returned.
     */
    bool getCost(int x, int y, double& cost);

        /**
     * Allows to request the cost of the cell within the DStar-Lite map (world/root frame).
     * If the cell (x,y) has not been added yet false will be returned.
     */
    bool getCostWorld(double x, double y, double& cost);
    
    inline void resetStatistics() {
        mStatistics.reset();
    }
    
    inline struct Statistics getStatistics() {
        return mStatistics;
    }
    
    inline envire::TraversabilityGrid* getRootTravMap() {
        return mTravGrid;
    }

    inline void setRemoveObstaclesRadius(double radius) {
        mRemoveObstaclesRadius = radius;
    }
  
    
 private:     
    dstar_lite::DStarLite* mDStarLite;
    /** First received trav. grid, used as the world grid. */
    envire::TraversabilityGrid* mTravGrid;
    envire::Environment mEnv;
    struct Statistics mStatistics;
    /** To be able to detach the new grid during each map update. */
    envire::TraversabilityGrid* mNewTravGrid;
    envire::FrameNode* mNewTravFrameNode;
    
    Eigen::Vector2i mStartPos;
    Eigen::Vector2i mGoalPos;  
    double mRemoveObstaclesRadius;
};
}

#endif

