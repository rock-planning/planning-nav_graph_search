#define BOOST_TEST_MODULE NavGraphSearch 
#include <boost/test/included/unit_test.hpp>

#include <stdlib.h>     /* srand, rand */
#include <stdio.h>
#include <time.h>       /* time */

#include <sstream>

#include <nav_graph_search/traversability_map.hpp>
#include <nav_graph_search/dstar_lite.hpp>

using namespace nav_graph_search;
using namespace envire;

size_t width = 200;
size_t height = 200;
TraversabilityMap trav_map(width, height, 0, 1); // Fill with class 0, size of each cell / pixel is set to one meter.
TerrainClasses terrain_classes;

BOOST_AUTO_TEST_CASE( create_traversibility_map )
{
    // Add classes (0 to 16) to the map.
    srand (time(NULL));
    int class_value;
    for(unsigned int y = 0; y < height; ++y) {
        for(unsigned int x = 0; x < width; ++x) {
            class_value = rand() % TraversabilityMap::CLASSES_COUNT; // 0 to 15
            if(x == 0 || x == width-1 || y == 0 || y == height-1) {
              class_value = 0; // Represents an obstacle.
            }
            trav_map.setValue(x, y, class_value);
        }
    }
}

BOOST_AUTO_TEST_CASE( create_terrain_classes )
{
    terrain_classes.empty();
    struct TerrainClass terrain_class;
    for(int i=0; i<TraversabilityMap::CLASSES_COUNT; ++i) {
      terrain_class.in = terrain_class.out = i;
      terrain_class.cost = i; // Represents speed in m/sec
      terrain_class.margin = 0;
      std::stringstream ss;
      ss << i;
      terrain_class.name = ss.str();
      terrain_classes.push_back(terrain_class);
    }
}

/** Changes the traversability classes of 'width' cells. */
void randUpdateMap(DStarLite& dstar_lite) {
    int cell_x = 0, cell_y = 0;
    int new_class = 0;
    for(unsigned int i=0; i<width; i++) {
        cell_x = rand()%width;
        cell_y = rand()%height;
        new_class = rand()%TraversabilityMap::CLASSES_COUNT;
        dstar_lite.setTraversability(cell_x, cell_y, new_class);
    }
};

BOOST_AUTO_TEST_CASE( dstar_lite_planner_norobotsize_noinflatemax )
{
    DStarLite dstar_lite(trav_map, terrain_classes, 0, false);
    double path_cost = 0;
    int start_x, start_y, goal_x, goal_y;
    printf("%d map updates will be executed after each new start position\n", width);
    for(int i_goal=0; i_goal<10; ++i_goal) {
        goal_x = rand()%width;
        goal_y = rand()%height;
        for(int i_start=0; i_start<10; ++i_start) {   
            start_x = rand()%width;
            start_y = rand()%height;
            randUpdateMap(dstar_lite);
            BOOST_REQUIRE_NO_THROW(path_cost = dstar_lite.run(goal_x, goal_y, start_x, start_y));
            printf("Path from start (%d,%d) to goal (%d,%d) cost: %4.2f sec.\n", start_x, start_y, goal_x, goal_y, path_cost);  
        }
    }
}


