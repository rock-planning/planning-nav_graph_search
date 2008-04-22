#ifndef NAV_DSTAR_HPP
#define NAV_DSTAR_HPP

#include <vector>

namespace Nav {
    /** Objects of this class represent traversability maps. Traversability is
     * an integer value which can be represented on 4 bits (i.e. 16
     * traversability classes).
     */
    class TraversabilityMap
    {
        size_t m_xsize, m_ysize;

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

        /** Returns the traversability value for the cell at (x, y) */
        uint8_t getValue(size_t x, size_t y) const;
        /** Changes the traversability value for the cell at (x, y) */
        void setValue(size_t x, size_t y, uint8_t value);
    };

    class GridGraph;

    /** Objects of this class are used to iterate on GridGraph cells. 
     *
     * @see GridGraph::parentsBegin GridGraph::parentsEnd GridGraph::neighboursBegin GridGraph::neighboursEnd
     */
    class NeighbourIterator
    {
        /** The underlying graph */
        GridGraph* m_graph;

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
        NeighbourIterator()
            : m_graph(0), m_neighbour(END_NEIGHBOUR) {}
        NeighbourIterator(GridGraph& graph, int x, int y, int mask);

        int getMask() const { return m_mask; }

        /** The index of the neighbour we are currently visiting. Useful mostly
         * for debugging purposes */
        int getNeighbour() const { return m_neighbour; }

        /** The position of the node on which we are iterating */
        int nodeX() const { return m_x; }
        int nodeY() const { return m_y; }

        /** The position of the currently-iterated neighbour */
        int x() const { return m_x + m_x_offsets[m_neighbour]; }
        int y() const { return m_y + m_y_offsets[m_neighbour]; }

        /** The value of the currently-iterated neighbour */
        float& value();

        /** Advance the iteration one step */
        NeighbourIterator& operator++()
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
        bool operator == (NeighbourIterator const& other) const
        {
            return (m_neighbour == other.m_neighbour
                    || (m_neighbour >= END_NEIGHBOUR && other.m_neighbour >= END_NEIGHBOUR)
                && m_graph == other.m_graph
                && m_x == other.m_x
                && m_y == other.m_y);
        }
        /** The inverse of ==
         */
        bool operator != (NeighbourIterator const& other) const { return !(*this == other); }
    };


    /** Objects of this class represent a DAG based on regular grids. Each node
     * in the graph has 8 neighbours, and given a node, only itss parents are
     * available.
     *
     * Finally, one single floating-point value can be stored per node.
     */
    class GridGraph
    {
    public:
        typedef NeighbourIterator iterator;

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

    private:
        size_t m_xsize, m_ysize;

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
        GridGraph(size_t width, size_t height);

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
        /** Returns the past-the-end NeighbourIterator object to iterate on the
         * parents of cells.
         */
        iterator parentsEnd();
        /** Returns a NeighbourIterator object which allows to iterate on the
         * neighbours of (x, y). Neighbours are the cells directly adjacent to
         * the considered cell.
         */
        iterator neighboursBegin(size_t x, size_t y);
        /** Returns the past-the-end NeighbourIterator object to iterate on the
         * neighbours of cells. See neighboursBegin.
         */
        iterator neighboursEnd();
    };

    class DStar
    {
    };
}

#endif

