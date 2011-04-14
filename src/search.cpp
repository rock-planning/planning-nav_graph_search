#include "search.hpp"

using namespace nav_graph_search;

namespace nav_graph_search {
    static const float DIAG_FACTOR = sqrt(2);
}

Search::Search(int width, int height)
    : m_graph(width, height, std::numeric_limits<float>::max())
    , m_initialized(false)
    , m_goal_x(-1), m_goal_y(-1)
    , m_start_x(-1), m_start_y(-1)
{
}

void Search::updated(int x, int y)
{}

TraversabilitySearch::TraversabilitySearch(
        TraversabilityMap& map,
        TerrainClasses const& classes)
    : Search(map.xSize(), map.ySize())
    , m_map(map)
{
    if (classes.empty())
    {
        for (int i = 0; i < TraversabilityMap::CLASSES_COUNT; ++i)
            m_cost_of_class[i] = TraversabilityMap::CLASSES_COUNT + 1 - i;
    }
    else
    {
        float map_scale = map.getScale();

        for (int i = 0; i < TraversabilityMap::CLASSES_COUNT; ++i)
            m_cost_of_class[i] = 1000000;

        for (TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); ++it)
        {
            float speed = it->cost;
            if (speed == 0)
                m_cost_of_class[it->out] = 1000000;
            else
                m_cost_of_class[it->out] = map_scale / speed;

            std::cerr << " class " << it->out << " has cost " << m_cost_of_class[it->out] << std::endl;
        }
    }
}

float TraversabilitySearch::costOfClass(int i) const { return m_cost_of_class[i]; }

float TraversabilitySearch::costOf(NeighbourConstIterator it) const
{
    float a = m_cost_of_class[m_map.getValue(it.sourceX(), it.sourceY())];
    float b = m_cost_of_class[m_map.getValue(it.x(), it.y())];

    if (it.getNeighbour() & GridGraph::DIR_STRAIGHT)
        return a + b;

    uint8_t next_neighbour = it.getNeighbour() << 1;
    uint8_t prev_neighbour = it.getNeighbour() >> 1;
    if (! next_neighbour)
        next_neighbour = GridGraph::RIGHT;
    else if (! prev_neighbour)
        prev_neighbour = GridGraph::BOTTOM_RIGHT;

    NeighbourConstIterator next = m_graph.getNeighbour(it.sourceX(), it.sourceY(), next_neighbour);
    NeighbourConstIterator prev = m_graph.getNeighbour(it.sourceX(), it.sourceY(), prev_neighbour);

    float c = m_cost_of_class[m_map.getValue(next.x(), next.y())];
    float d = m_cost_of_class[m_map.getValue(prev.x(), prev.y())];
    return (a + b + c + d) / 2 * nav_graph_search::DIAG_FACTOR;
}

void TraversabilitySearch::setTraversability(int x, int y, int klass)
{
    m_map.setValue(x, y, klass);
    updated(x, y);
}


