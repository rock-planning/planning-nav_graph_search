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


#include <nav_graph_search/search.hpp>
#include <nav_graph_search/traversability_map.hpp>

namespace dstar_lite {
class DStarLite;
}

namespace nav_graph_search {

/** 
 * Using the sources within src/dstar_lite and the class TraversabilitySearch
 * to realize DStar-Lite path planning. TraversabilitySearch is just used to 
 * generating and updating the cost map regarding the footprint-size. 
 * The cost values are used (which requires smaller modifications)
 * to build up the internal DStar-Lite cost map.\n
 * In the current version the grid size should not exceed ~ 200x200 and the cost values
 * should be close to 1, otherwise the heuristic becomes useless. So, the robot speed 
 * should be described by m/sec.
 */
class DStarLite : public TraversabilitySearch
{
 public:
    /**
     * Creates the cost map by passing the parameters to TraversabilitySearch and
     * allocates the internal dstar_lite::DStarLite object.
     * \param map Contains the classes for each grid.
     * \param classes Maps the classes to a value representing the speed
     * of the robot traversing the cell (should be m/sec). The passed map and the classes 
     * are used to generate a cost map containing (for each cell) the time for traversing.
     * The internal DStar-Lite cost map is synchronized with this cost map (containing
     * some necessary adaption, e.g. >= 1000000 is mapped to -1 to represent obstacles).
     * \param robotSize (footsize) If > 0 changing the class/cost of a cell will influence 
     * the surrounded cells as well (cells under the robot / under the foot). All
     * cells will be set to an average or the max value.
     * \param inflateMax If true the obstacles will be grown using the full robot size
     * (half would be enough?). Otherwise the robot will just try not to get to close
     * to obstacles.
     */
    DStarLite(TraversabilityMap& map, 
            TerrainClasses const& classes, 
            int robotSize = 0, 
            bool inflateMax = false);

    /**
     * Deallocates the internal dstar_lite::DStarLite object.
     */
    ~DStarLite();

    /** 
     * Not used, the internal DStar-Lite implementation uses its own heuristic function.
     */
    virtual double getHeuristic(int to_x, int to_y, int from_x, int from_y);

    /** 
     * Function is called by 'setTraversability()' and is used to updates the internal DStar-Lite map.
     */
    virtual void updated(int x, int y);

    /**
     * Run the algorithm for the given goal and start point
     * \return The cost (time) from the start point to the goal point. 
     * @throw std::runtime_error if no solution can be found
     */
    virtual double run(int goal_x, int goal_y, int start_x, int start_y);

 private:
    dstar_lite::DStarLite* m_dstarLite;
    bool m_dstarLiteInitialized;

    /**
     * Initializes the dstar_lite::Dstar once.
     * Transfers the cost values (time) of the current nav_graph_search m_costMap to 
     * the internal dstar_lite map.\n
     * All values are incremented (dstar_lite values may not lie in the range of [0,1)) 
     * and values >= 1.000.000 are mapped to -1 which represents obstacles.
     */
    void initializeDstarLite(int goal_x, int goal_y, int start_x, int start_y);

    /**
     * The DStar-Lite implementation uses -1 to represent obstacles
     * and [0,1) values are not allowed.
     */
    inline double navGraph2DStarLiteCosts(double nav_graph_cost) {
        return nav_graph_cost >= 1000000 ? -1 : ++nav_graph_cost;
    }

    static double dStarLite2NavGraphCosts(double dstar_lite_cost) {
        return dstar_lite_cost < 0 ? 1000000 : --dstar_lite_cost;
    }
};
}

#endif

