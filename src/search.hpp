#ifndef NAV_GRAPH_SEARCH_ALGORITHM_HPP
#define NAV_GRAPH_SEARCH_ALGORITHM_HPP

#include <nav_graph_search/traversability_map.hpp>
#include <nav_graph_search/grid_graph.hpp>

namespace nav_graph_search {
    /** Base class for all search algorithms implemented in this library
     *
     * This base class can be used for heuristic and non-heurstic as well as
     * incremental and non-incremental planners.
     *
     * Heuristic planners might want to reimplement the getHeuristic method.
     *
     * Incremental planners will need to reimplement the updated() method
     */
    class Search
    {
    protected:
        /** The GridGraph object we use to store the algorithm state. The float
         * value of a node in this graph stores the cost of the path from that
         * cell to the goal
         */
        GridGraph m_graph;

        /** True if run() has been called at least once with a given goal */
        bool m_initialized;

        /** The goal during the last run */
        int m_goal_x, m_goal_y;

        /** The start point during the last run */
        int m_start_x, m_start_y;

    public:
        Search(int width, int height);

        int getStartX() const { return m_start_x; }
        int getStartY() const { return m_start_y; }

        int getGoalX() const { return m_goal_x; }
        int getGoalY() const { return m_goal_y; }

        /** Returns an acceptable heuristic distance from (from_x, from_y) to
         * (to_x, to_y)
         *
         * The default implementation returns 0. Heuristic search algorithm
         * implementation should make sure that their default implementation is
         * more meaningful
         */
        virtual double getHeuristic(int to_x, int to_y, int from_x, int from_y);

        /** The graph object which is used to store the algorithm result */
        GridGraph& graph() { return m_graph; }

        /** The graph object which is used to store D*'s results */
        GridGraph const& graph() const { return m_graph; }

        /** Announce that the given cell has been updated in the underlying
         * map
         *
         * This default implementation does nothing. It should be reimplemented
         * only for incremental algorithms
         */
        virtual void updated(int x, int y);

        /** Run the algorithm for the given goal and start point
         *
         * Returns the cost from the start point to the goal point
         *
         * @throw std::runtime_error if no solution can be found
         */
        virtual double run(int goal_x, int goal_y, int start_x, int start_y) = 0;
    };

    /** Base class for algorithms that are based on a TraversabilityMap
     */
    class TraversabilitySearch : public Search
    {
    protected:
        float m_cost_of_class[TraversabilityMap::CLASSES_COUNT];

        /** Minimal cost in m_cost_of_class. This is used for the default
         * implementation of getHeuristic
         */
        double m_min_class_cost;

        /** The underlying map we are acting on */
        TraversabilityMap& m_map;

    public:
        TraversabilitySearch(
                TraversabilityMap& map,
                TerrainClasses const& classes = TerrainClasses());

        /** Returns the basic cost associated with the given terrain class */
        float costOfClass(int i) const;

        /** Computes the cost of crossing the edge represented by \c it
         *
         * The strategy is the following:
         * <ul>
         *  <li>use (a + b) / 2, where a and b are the costs of the source and
         *     target cells, if we are going straight
         *  <li>use (a + b + c + d) / 2, where a and b are the costs of the source
         *     and target cells and c and d are the costs of the two adjacent cells.
         *     This is used if we are going in diagonal.
         * </ul>
         */
        float costOf(NeighbourConstIterator it) const;

        /** Sets the traversability class to \c klass for the given cell */
        void setTraversability(int x, int y, int klass);

        /** Returns an acceptable heuristic distance from (from_x, from_y) to
         * (to_x, to_y)
         *
         * The default implementation returns the euclidian distance between the
         * two points, times the lowest cost of all the classes. Heuristic
         * search algorithm implementation should make sure that their default
         * implementation is more meaningful
         */
        virtual double getHeuristic(int to_x, int to_y, int from_x, int from_y)
        {
            int dx = to_x - from_x;
            int dy = to_y - from_y;
            double d = sqrt(dx * dx + dy * dy);
            return m_min_class_cost * d;
        }
    };
}

#endif

