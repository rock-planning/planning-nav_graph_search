#ifndef NAV_DSTAR_HPP
#define NAV_DSTAR_HPP

#include <vector>
#include <map>
#include <boost/type_traits.hpp>

namespace Nav {
    /** Basic tools for maps which are regular grids */
    class GridMap
    {
    protected:
        size_t m_xsize, m_ysize;

    public:
        GridMap(size_t xsize, size_t ysize)
            : m_xsize(xsize), m_ysize(ysize) { }


        /** The size, in cells, in the X direction */
        size_t xSize() const { return m_xsize; }
        /** The size, in cells, in the Y direction */
        size_t ySize() const { return m_ysize; }
        /** A numeric ID for the cell at (x, y) */
        size_t getCellID(size_t x, size_t y) const { return y * m_xsize + x; }
    };

    /** Objects of this class represent traversability maps. Traversability is
     * an integer value which can be represented on 4 bits (i.e. 16
     * traversability classes).
     */
    class TraversabilityMap : public GridMap
    {
        /** The vector of values. Traversability is encoded on 4-bits fields, which
         * means that one uint8_t stores two cells. Moreover, values are stored
         * X-first, which means that the value for the cell (x, y) is at (y *
         * xsize + x).
         */
        std::vector<uint8_t> m_values;

    public:
        /** How much classes can be stored in this map */
        static const int CLASSES_COUNT = 16;

        /** Creates a new map with the given size in the X and Y axis */
        TraversabilityMap(size_t xsize, size_t ysize);

        /** Returns the traversability value for the cell with the given ID */
        uint8_t getValue(size_t id) const;
        /** Changes the traversability value for the cell with the given ID */
        void setValue(size_t id, uint8_t value);
        /** Returns the traversability value for the cell at (x, y) */
        uint8_t getValue(size_t x, size_t y) const;
        /** Changes the traversability value for the cell at (x, y) */
        void setValue(size_t x, size_t y, uint8_t value);
    };

    class GridGraph;

    /** Base class for iteration around GridGraph cells. It is the common part
     * of NeighbourIterator and NeighbourConstIterator, the two classes you should
     * be using.
     *
     * @see GridGraph::parentsBegin GridGraph::parentsEnd GridGraph::neighboursBegin GridGraph::neighboursEnd
     */
    template<typename GraphClass>
    class NeighbourGenericIterator
    {
        friend class NeighbourGenericIterator<typename boost::add_const<GraphClass>::type>;

    protected:
        /** The underlying graph */
        GraphClass* m_graph;

        /** The position of the node whose neighbours we are visiting */
        int m_x, m_y;

        /** A bitfield which describes the remaining neighbours we should
         * visit. The iterator will not stop on neighbours for which a 0
         * is set in the mask.
         *
         * See the code in findNextNeighbour
         */
        int m_mask;

        /** the neighbour we are currently visiting. It is an integer between 1
         * and 9 (inclusive). The values between 1 and 8 describe the actual
         * relation between the central node in (m_x, m_y) and the currently
         * visited neighbour. 9 is the end of iteration.
         */
        int m_neighbour;

        static const int END_NEIGHBOUR = 9;
        static const int m_x_offsets[9];
        static const int m_y_offsets[9];

        /** Updates m_neighbour and m_mask to advance the iterator to the next
         * cell which should be visited, or to the end of iteration if no
         * cell remains
         */
        void findNextNeighbour();

    public:
        NeighbourGenericIterator()
            : m_graph(0), m_neighbour(END_NEIGHBOUR) {}
        NeighbourGenericIterator(GraphClass& graph, int x, int y, int mask);
        NeighbourGenericIterator(NeighbourGenericIterator<typename boost::remove_const<GraphClass>::type> const& other)
        {
            m_graph     = other.m_graph;
            m_x         = other.m_x;
            m_y         = other.m_y;
            m_mask      = other.m_mask;
            m_neighbour = other.m_neighbour;
        }

        int getMask() const { return m_mask; }

        /** The neighbour we are currently visiting, as a value defined
         * in GridGraph::RELATIONS */
        int getNeighbour() const { return 1 << (m_neighbour - 1); }

        /** The position of the node on which we are iterating */
        int sourceX() const { return m_x; }
        int sourceY() const { return m_y; }

        /** The position of the currently-iterated neighbour */
        int x() const { return m_x + m_x_offsets[m_neighbour]; }
        int y() const { return m_y + m_y_offsets[m_neighbour]; }

        /** Advance the iteration one step */
        NeighbourGenericIterator<GraphClass>& operator++()
        {
            ++m_neighbour;
            m_mask >>= 1;
            findNextNeighbour();
            return *this;
        };

        /** True if *this and \c other represent the same position in
         * iteration. It means that they both point to the same underlying cell
         * in the graph \b and are iterating around the same center node
         */
        bool operator == (NeighbourGenericIterator const& other) const
        {
            return (m_neighbour == other.m_neighbour
                    || (m_neighbour >= END_NEIGHBOUR && other.m_neighbour >= END_NEIGHBOUR)
                && m_graph == other.m_graph
                && m_x == other.m_x
                && m_y == other.m_y);
        }
        /** The inverse of ==
         */
        bool operator != (NeighbourGenericIterator const& other) const { return !(*this == other); }
    };

    template<typename GraphClass>
    const int NeighbourGenericIterator<GraphClass>::m_x_offsets[9] = { 0, 1, 1, 0, -1, -1, -1,  0,  1 };
    template<typename GraphClass>
    const int NeighbourGenericIterator<GraphClass>::m_y_offsets[9] = { 0, 0, 1, 1,  1,  0, -1, -1, -1 };

    class NeighbourIterator : public NeighbourGenericIterator<GridGraph>
    {
    public:
        NeighbourIterator()
            : NeighbourGenericIterator<GridGraph>() {}
        NeighbourIterator(GridGraph& graph, int x, int y, int mask)
            : NeighbourGenericIterator<GridGraph>(graph, x, y, mask) {}
        NeighbourIterator(NeighbourGenericIterator<GridGraph> const& source)
            : NeighbourGenericIterator<GridGraph>(source) {}

        /** The value of the currently-iterated neighbour */
        float& value();
        float value() const;
    };

    class NeighbourConstIterator : public NeighbourGenericIterator<GridGraph const>
    {
    public:
        NeighbourConstIterator()
            : NeighbourGenericIterator<GridGraph const>() {}
        NeighbourConstIterator(GridGraph const& graph, int x, int y, int mask)
            : NeighbourGenericIterator<GridGraph const>(graph, x, y, mask) {}
        NeighbourConstIterator(NeighbourIterator const& source)
            : NeighbourGenericIterator<GridGraph const>(source) {}

        /** The value of the currently-iterated neighbour */
        float value() const;
    };

    /** Objects of this class represent a DAG based on regular grids. Each node
     * in the graph has 8 neighbours, and given a node, only itss parents are
     * available.
     *
     * Finally, one single floating-point value can be stored per node.
     */
    class GridGraph : public GridMap
    {
    public:
        typedef NeighbourIterator iterator;
        typedef NeighbourConstIterator const_iterator;

        /** Enum which describes the mapping between the bits in m_parents and
         * the actual neighbour direction.
         */
        enum RELATIONS {
            RIGHT        = 1,
            TOP_RIGHT    = 2,
            TOP          = 4,
            TOP_LEFT     = 8,
            LEFT         = 16,
            BOTTOM_LEFT  = 32,
            BOTTOM       = 64,
            BOTTOM_RIGHT = 128
        };

        static const int DIR_STRAIGHT = RIGHT | TOP | LEFT | BOTTOM;
        static const int DIR_DIAGONAL = TOP_RIGHT | TOP_LEFT | BOTTOM_LEFT | BOTTOM_RIGHT;

    private:
        /** Array which encodes the parents of each node. The array is stored
         * row-first (i.e. the value for (x, y) is at (y * m_xsize + x).
         *
         * The parents are encoded as a bitfield. The least significant bit is
         * the node on the same line than the considered node, and then we turn
         * counter-clockwise. See the RELATIONS enum.
         *
         */
        std::vector<uint8_t> m_parents;


        /** Array which encodes the parents of each node. The array is stored
         * row-first (i.e. the value for (x, y) is at (y * m_xsize + x).
         */
        std::vector<float> m_values;

    public:
        GridGraph(size_t width, size_t height, float value = 0);

        /** Clears all edges in the graph, and initialize the floating-point
         * value to the given \c new_value */
        void clear(float new_value);

        /** Returns the floating point value stored for the (x, y) cell */
        float    getValue(size_t x, size_t y) const;
        /** Returns the floating point value stored for the (x, y) cell */
        float&   getValue(size_t x, size_t y);

        /** Returns the bit-mask describing what are the parents of the cell
         * at (x, y). See RELATIONS.
         */
        uint8_t  getParents(size_t x, size_t y) const;
        /** Returns the bit-mask describing what are the parents of the cell
         * at (x, y). See RELATIONS.
         */
        uint8_t& getParents(size_t x, size_t y);
        /** Adds a new parent for the cell at (x, y). See RELATIONS for the
         * set of possible values.
         */
        void     setParent(size_t x, size_t y, uint8_t new_parent);
        /** Removes a parent for the cell at (x, y). See RELATIONS for the set
         * of possible values.
         */
        void     clearParent(size_t x, size_t y, uint8_t old_parent);

        /** Returns a NeighbourIterator object which allows to iterate on the
         * parents of (x, y). See also getParents.
         */
        iterator parentsBegin(size_t x, size_t y);
        /** Returns a NeighbourIterator object which allows to iterate on the
         * parents of (x, y). See also getParents.
         */
        const_iterator parentsBegin(size_t x, size_t y) const;
        /** Returns the past-the-end NeighbourIterator object to iterate on the
         * parents of cells.
         */
        iterator parentsEnd() const;
        /** Returns a NeighbourIterator object which allows to iterate on the
         * neighbours of (x, y). Neighbours are the cells directly adjacent to
         * the considered cell.
         */
        iterator neighboursBegin(size_t x, size_t y);
        /** Returns a NeighbourIterator object which allows to iterate on the
         * neighbours of (x, y). Neighbours are the cells directly adjacent to
         * the considered cell.
         */
        const_iterator neighboursBegin(size_t x, size_t y) const;
        /** Returns the past-the-end NeighbourIterator object to iterate on the
         * neighbours of cells. See neighboursBegin.
         */
        iterator neighboursEnd() const;

        /** Returns the iterator pointing to the given neighbour of (x, y) */
        NeighbourConstIterator getNeighbour(size_t x, size_t y, int neighbour) const;
    };

    /** An implementation of the plain D* algorithm */
    class DStar
    {
        /** The underlying map we are acting on */
        TraversabilityMap const& m_map;

        /** The GridGraph object we use to store the algorithm state */
        GridGraph m_graph;

        std::map<float, int> m_open_list;

        /** The current goal */
        int m_goal_x, m_goal_y;

    public:
        DStar(TraversabilityMap const& map);

        /** The graph object which is used to store D*'s results */
        GridGraph const& graph() const;

        /** Initializes the algorithm for the given position and goal */
        void initialize(int goal_x, int goal_y, int pos_x, int pos_y);

        /** Announce that the given cell has been updated in the traversability
         * map */
        void updated(int x, int y);

        /** Update the trajectories for the given position */
        void update(int pos_x, int pos_y);

        /** Computes the cost of traversing a cell of the given class
         * If p is the probability of being able to traverse the cell,
         * the cost is
         *   
         *   2**((1-p)/s(p))
         *
         * where 
         *   s(p) = p * (COST_1 - COST_0) + COST_0
         *
         * The basic idea of this cost function is to base itself on the idea
         * of 
         *
         *   2**((1-p)/s)
         *
         * i.e. having an exponential growth in term of probabilities. The growth
         * rate \c s is then made function of p, so that the cost function grows
         * rapidly for low probabilities (high-cost areas) and slowly for
         * low cost areas.
         *
         * The values for COST_1 and COST_0 are defined in dstar.cc
         */
        static float costOfClass(int klass);

        /** The cost of traversing the given cell */
        float costOf(size_t x, size_t y) const;

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
    };
}

#endif

