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

#include <envire/maps/Grids.hpp>
#include <nav_graph_search/terrain_classes.hpp>
#include <base/geometry/spline.h>

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
 * In the current version the cost values should be close to 0 (nav_graph cost map)
 * respectively close to 1 (dstar_lite cost map), otherwise the heuristic becomes useless.
 * This can be achieved by using a smaller grid size or different speed units e.g. m/minute.
 */
class DStarLite
{
 public:
    /**
     * Creates the cost map by passing the parameters to TraversabilitySearch and
     * allocates the internal dstar_lite::DStarLite object.
     * \param classes Maps the classes to a value representing the speed
     * of the robot traversing the cell (should be described by m/time). The passed map and the classes 
     * are used to generate a cost map containing (for each cell) the time for traversing.
     * The internal DStar-Lite cost map is synchronized with this cost map (containing
     * some necessary adaption: >= 1000000 are mapped to -1 to represent obstacles
     * and all other costs are incremented by 1 to avoid costs between 0 and 1 [0,1)).
     */
    DStarLite(TerrainClasses const& classes);

    /**
     * Deallocates the internal dstar_lite::DStarLite object.
     */
    ~DStarLite();

    /**
     * Convenience function. 
     * Transforms start and goal to local coordinates and calles run();
     * */
    bool run(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const Eigen::Affine3d &fromWorld);
    
    /**
     * Run the algorithm for the given goal and start point
     * \return If a path could be computed
     */
    bool run(int goal_x, int goal_y, int start_x, int start_y);

    /**
     * Updates the Dstar with the given map.
     * */
    void updateTraversabilityMap(envire::TraversabilityGrid *grid);
    
    /** 
     * Updates the traversability class to \c klass for the given cell.
     */
    void updateTraversability(int x, int y, int klass);

    std::vector<base::geometry::Spline<3> > getTrajectory(const Eigen::Affine3d &gridToWorld) const;
    
    std::vector<Eigen::Vector2i> getLocalTrajectory() const;
 private:
    bool needsInit; 
     
    envire::TraversabilityGrid *curGrid;
    
    ///maps terrain class to cost
    std::vector<TerrainClass> costMap;
    dstar_lite::DStarLite* m_dstarLite;
    
    Eigen::Vector2i start;
    Eigen::Vector2i goal;    
};
}

#endif

