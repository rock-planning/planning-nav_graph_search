#ifndef NAV_DSTAR_HPP
#define NAV_DSTAR_HPP

#include <nav_graph_search/point.hpp>
#include <nav_graph_search/pool_allocator.hpp>
#include <nav_graph_search/traversability_map.hpp>
#include <nav_graph_search/grid_graph.hpp>
#include <nav_graph_search/search.hpp>

#include <map>
#include <stdexcept>

namespace nav_graph_search {
    /** An implementation of the plain D* algorithm. DStar::costOfClass is the
     * method which transforms traversability value into a floating-point cost
     * value
     */
    class DStar : public TraversabilitySearch
    {
    public:
        struct internal_error : public std::runtime_error
        {
            internal_error(std::string const& msg)
                : std::runtime_error(msg) {}
        };

        struct Cost {
            float value;

            Cost(float value) : value(value) {}

            bool operator < (Cost const& other) const  { return value - other.value < -0.0001; }
            bool operator <= (Cost const& other) const { return !(*this > other); }
            bool operator > (Cost const& other) const  { return value - other.value > 0.0001; }
            bool operator >= (Cost const& other) const { return !(*this < other); }
            bool operator == (Cost const& other) const { return std::fabs(value - other.value) <= (value / 10000); }
            bool operator != (Cost const& other) const { return !(*this == other); }
            Cost operator - (Cost const& other) const
            { return Cost(value - other.value); }
            Cost operator + (Cost const& other) const
            { return Cost(value + other.value); }
        };

    private:
        typedef std::multimap<Cost, PointID, std::less<Cost>,
                pool_allocator< std::pair<Cost, PointID> > > OpenFromCost;
        OpenFromCost m_open_from_cost;
        typedef std::map<PointID, Cost, std::less<PointID>,
                pool_allocator< std::pair<PointID, Cost> > > OpenFromNode;
        OpenFromNode m_open_from_node;

        void run(int goal_x, int goal_y, int start_x, int start_y, double max_cost);

    public:
        DStar(TraversabilityMap& map, TerrainClasses const& classes = TerrainClasses(), int robotSize = 0, bool inflateMax = false);

        /* Insert the following point in the open list, using the given value
         * as ordering value
         */
        Cost insert(int x, int y, Cost value);

        /** Initializes the algorithm for the given goal
         *
         * You usually don't have to call this directly. run() will call it
         * when needed
         */
        void initialize(int goal_x, int goal_y);

        /** Announce that the given cell has been updated in the traversability
         * map */
        void updated(int x, int y);

        /** Run the algorithm for the given goal and start point
         *
         * Returns the cost from the start point to the goal point
         *
         * @throw std::runtime_error if no solution can be found
         */
        virtual double run(int goal_x, int goal_y, int start_x, int start_y);

        /** Continue expanding nodes until the lowest cost node to expand has a
         * cost greater than \c cost
         */
        virtual void expandUntil(double max_cost);

        /** True if \c it points to a cell which has never been considered by
         * the algorithm */
        bool isNew(NeighbourConstIterator it) const;

        /** True if the pointed-to cell of \c it is currently in the open set */
        bool isOpened(NeighbourConstIterator it) const;

        /** True if \c x and \c y define a cell which has never been considered
         * by the algorithm */
        bool isNew(int x, int y) const;

        /** True if (\c x, \c y) is currently in the open set */
        bool isOpened(int x, int y) const;

        /** True if the pointed-to cell of \c it is neither new nor opened */
        bool isClosed(NeighbourConstIterator it) const
        { return isClosed(it.x(), it.y()); }

        /** True if (\c x, \c y) is neither new nor opened */
        bool isClosed(int x, int y) const { return !isNew(x, y) && !isOpened(x, y); }

        /** Returns the cost of (x, y) as stored in the open list. If the node
         * is not in the open list, the boolean returned is false. Otherwise,
         * the boolean is true and the float value is the new cost of the node
         *
         * If \c check_consistency is true, then the method performs a
         * consistency check on the open list. It is quite costly and should
         * be used only for testing purposes
         */
        std::pair<float, bool> updatedCostOf(int x, int y, bool check_consistency = false) const;

        /** Checks that the current solution is consistent. It raises internal_error
         * if it is not the case */
        bool checkSolutionConsistency() const;
    };
}

#endif

