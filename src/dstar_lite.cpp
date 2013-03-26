#include "dstar_lite.hpp"
#include "dstar_lite/DStarLite.h"

#include <stdexcept>

#include <base/logging.h>

using namespace std;

namespace nav_graph_search {

DStarLite::DStarLite(TraversabilityMap& map, TerrainClasses const& classes, int robotSize, bool inflateMax) : 
        TraversabilitySearch(map, classes, robotSize, inflateMax), 
        m_dstarLite(NULL), 
        m_dstarLiteInitialized(false),
        m_using_env_map_directly(false)
{
    m_dstarLite = new dstar_lite::DStarLite();   
}

DStarLite::DStarLite(envire::TraversabilityGrid const& envire_map, std::string const& band_name,
        TerrainClasses const& classes) :  
            TraversabilitySearch(envire_map, band_name, classes),   
            m_dstarLite(NULL), 
            m_dstarLiteInitialized(false),
            m_using_env_map_directly(true) 
{

    m_dstarLite = new dstar_lite::DStarLite(); 
    const envire::TraversabilityGrid::ArrayType &gridData(envire_map.getGridData());

    // Fill dstar lite map with the converted time costs using the received envire trav map.
    for(unsigned int y=0; y < envire_map.getCellSizeY(); ++y) {
        for(unsigned int x=0; x < envire_map.getCellSizeX(); ++x) {
            setTraversability(x, y, gridData[x][y]);
        }
    }
}

DStarLite::~DStarLite() {
    if(m_dstarLite != NULL) {
        delete m_dstarLite;
        m_dstarLite = NULL;
    }
}

double DStarLite::getHeuristic(int to_x, int to_y, int from_x, int from_y) {
    return 0.0;
}

void DStarLite::updated(int x, int y) { 
    // Costs have already been adapted in setTraversablity() (according to the footprint).
    double cost = m_costMap.getValue(x,y); 
    m_dstarLite->updateCell(x,y,navGraph2DStarLiteCosts(cost)); // Updates dstar_lite map.
}

double DStarLite::run(int goal_x, int goal_y, int start_x, int start_y) {
    initializeDstarLite(goal_x, goal_y, start_x, start_y); // Done once.
    
    if(goal_x != m_goal_x || goal_y != m_goal_y) {
        LOG_INFO("Received new goal position (%d,%d)", goal_x, goal_y);
        m_dstarLite->updateGoal(goal_x, goal_y);
        m_goal_x = goal_x;
        m_goal_y = goal_y;
    }

    if(start_x != m_start_x || start_y != m_start_y) {
        m_dstarLite->updateStart(start_x, start_y);
        m_start_x = start_x;
        m_start_y = start_y;
    }

    if(!m_dstarLite->replan()) {
        LOG_WARN("Path could not be found, goal will be reset once");
        m_dstarLite->resetGoal();
        if(!m_dstarLite->replan()) {
            throw std::runtime_error("No path could be found using DStar-Lite");
        }
    }

    // Returns the costs (time) for the current path using the nav-graph-values.
    return m_dstarLite->getPathCost(&dStarLite2NavGraphCosts);
}

void DStarLite::setTraversability(int x, int y, int klass) {
    if(m_using_env_map_directly) {
        float time_cost = m_cost_of_class[klass];
        m_dstarLite->updateCell(x,y,navGraph2DStarLiteCosts(time_cost)); 
    } else {
        TraversabilitySearch::setTraversability(x, y, klass);
    }
}

//////////////////////////////////////// PRIVATE ////////////////////////////////////////
void DStarLite::initializeDstarLite(int goal_x, int goal_y, int start_x, int start_y) {
    if(m_dstarLiteInitialized)
        return;

    m_dstarLite->init(start_x, start_y, goal_x, goal_y);
    m_goal_x = goal_x;
    m_goal_y = goal_y;
    m_start_x = start_x;
    m_start_y = start_y;

    double cost = 0.0;
    for(int y=0; y < m_costMap.ySize(); ++y) {
        for(int x=0; x < m_costMap.xSize(); ++x) {
            cost = m_costMap.getValue(x,y);
            m_dstarLite->updateCell(x,y,navGraph2DStarLiteCosts(cost));
        }
    }

    m_dstarLiteInitialized = true;
}

} // end namespace nav_graph_search


