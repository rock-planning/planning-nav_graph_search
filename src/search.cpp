#include "search.hpp"
#include <algorithm>
#include <cstdio>

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

double Search::getHeuristic(int to_x, int to_y, int from_x, int from_y)
{
    return 0;
}

void Search::updated(int x, int y)
{}

TraversabilitySearch::TraversabilitySearch(
        TraversabilityMap& map,
        TerrainClasses const& classes, int robotSize, bool inflateMax)
    : Search(map.xSize(), map.ySize())
    , m_classMap(map)
    , m_footPrint(robotSize)
    , m_costMap(map.xSize(), map.ySize(), std::numeric_limits<float>::max())
    , m_inflateMax(inflateMax)
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

    m_min_class_cost = *std::min_element(
            m_cost_of_class, 
            m_cost_of_class + TraversabilityMap::CLASSES_COUNT);
    
    if (m_footPrint == 0)
    {
        for( int y = 0; y < map.ySize(); ++y )
        {
            for( int x = 0; x < map.xSize(); ++x )
            {
                float cost = m_cost_of_class[m_classMap.getValue(x,y)];
                m_costMap.setValue(x, y,  cost);
            }
        }
    }
    else if (m_inflateMax)
    {
        for( int y = 0; y < map.ySize(); ++y )
        {
            for( int x = 0; x < map.xSize(); ++x )
            {
                float cost = findMax(x, y);
                m_costMap.setValue( x, y, cost);
            }
        }
    }
    else
    {
        for( int y = 0; y < map.ySize(); ++y )
        {
            for( int x = 0; x < map.xSize(); ++x )
            {
                float cost = findAver(x, y);
                m_costMap.setValue( x, y, cost);
            }
        }
    }
}

float TraversabilitySearch::costOfClass(int i) const { return m_cost_of_class[i]; }

float TraversabilitySearch::costOf(NeighbourConstIterator it) const
{
    float a = m_costMap.getValue(it.sourceX(), it.sourceY());
    float b = m_costMap.getValue(it.x(), it.y());

    if (it.getNeighbour() & GridGraph::DIR_STRAIGHT)
        return a + b;

    int next_neighbour = it.getNeighbour() << 1;
    int prev_neighbour = it.getNeighbour() >> 1;
    if (next_neighbour > GridGraph::BOTTOM_RIGHT)
        next_neighbour = GridGraph::RIGHT;
    else if (prev_neighbour == 0)
        prev_neighbour = GridGraph::BOTTOM_RIGHT;

    NeighbourConstIterator next = m_graph.getNeighbour(it.sourceX(), it.sourceY(), next_neighbour);
    NeighbourConstIterator prev = m_graph.getNeighbour(it.sourceX(), it.sourceY(), prev_neighbour);

    float c = m_costMap.getValue(next.x(), next.y());
    float d = m_costMap.getValue(prev.x(), prev.y());
    return (a + b + c + d) / 2 * nav_graph_search::DIAG_FACTOR;
}

float TraversabilitySearch::findAver(int x, int y) const{
    int xsize = m_classMap.xSize();
    int ysize = m_classMap.ySize();
    
    if( xsize > x + m_footPrint + 1 ) xsize = x + m_footPrint + 1;
    if( ysize > y + m_footPrint + 1 ) ysize = y + m_footPrint + 1;
    
    float ans = 0;
    int cnt = 0;
    
    for( int i = ((x-m_footPrint) < 0) ? 0 : (x-m_footPrint); i < xsize; ++i )
      for( int j = ((y-m_footPrint) < 0) ? 0 : (y-m_footPrint); j < ysize; ++j )
	if( (x - i) * (x - i) + (y - j) * (y - j) <= m_footPrint * m_footPrint + 1)
	{
	  ans += m_cost_of_class[m_classMap.getValue(i,j)];
	  cnt++;
	}
return ans / cnt;
}

float TraversabilitySearch::findMax(int x, int y) const{
    int xsize = m_classMap.xSize();
    int ysize = m_classMap.ySize();
    
    if( xsize > x + m_footPrint + 1 ) xsize = x + m_footPrint + 1;
    if( ysize > y + m_footPrint + 1 ) ysize = y + m_footPrint + 1;
    
    float ans = -std::numeric_limits<float>::max();
    
    for( int i = ((x-m_footPrint) < 0) ? 0 : (x-m_footPrint); i < xsize; ++i )
      for( int j = ((y-m_footPrint) < 0) ? 0 : (y-m_footPrint); j < ysize; ++j )
	if( (x - i) * (x - i) + (y - j) * (y - j) <= m_footPrint * m_footPrint + 1)
	{
	  float tmp = m_cost_of_class[m_classMap.getValue(i,j)];
	  if( ans < tmp ) ans = tmp;
	}
return ans;
}

void TraversabilitySearch::setTraversability(int x, int y, int klass)
{
    m_classMap.setValue(x, y, klass);
    
    int xsize = m_classMap.xSize();
    int ysize = m_classMap.ySize();
    
    if( xsize > x + m_footPrint + 1 ) xsize = x + m_footPrint + 1;
    if( ysize > y + m_footPrint + 1 ) ysize = y + m_footPrint + 1;
     
    for( int i = ((x-m_footPrint) < 0) ? 0 : (x-m_footPrint); i < xsize; ++i )
      for( int j = ((y-m_footPrint) < 0) ? 0 : (y-m_footPrint); j < ysize; ++j )
	if( (x - i) * (x - i) + (y - j) * (y - j) <= m_footPrint * m_footPrint + 1)
	{
	    if( !m_inflateMax ) m_costMap.setValue( i, j, findAver(i,j) );
	    else m_costMap.setValue( i, j, findMax(i,j) );
	}
    updated(x, y);
}
