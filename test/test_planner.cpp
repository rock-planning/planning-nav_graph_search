#define BOOST_TEST_MODULE NavGraphSearch 
#include <boost/test/included/unit_test.hpp>

#include <stdlib.h>     /* srand, rand */
#include <stdio.h>
#include <time.h>       /* time */
#include <sys/time.h>

#include <sstream>

#include <envire/maps/TraversabilityGrid.hpp>
#include <nav_graph_search/dstar_lite.hpp>
#include <envire/Core.hpp>

using namespace nav_graph_search;
using namespace envire;

size_t sizeX = 500;
size_t sizeY = 500;
int num_change_goal = 4;
int num_change_start = 10;
int numTerrainClasses = 10;

envire::Environment envfoo;

// Using random values for each cell, which is suboptimal for planning.
BOOST_AUTO_TEST_CASE( create_traversibility_map )
{
    // Add classes (0 to 16) to the map.
    srand (time(NULL));
    uint8_t class_value;
    
    envire::TraversabilityGrid *trGrid = new envire::TraversabilityGrid(sizeY, sizeY, 0.05, 0.05);
    envire::TraversabilityGrid::ArrayType &trData(trGrid->getGridData(TraversabilityGrid::TRAVERSABILITY));
    trData.resize(boost::extents[sizeY][sizeX]);
    
    for(unsigned int y = 0; y < sizeY; ++y) {
        for(unsigned int x = 0; x < sizeX; ++x) {
            class_value = rand() % numTerrainClasses; // 0 to 15
            if(x == 0 || x == sizeX-1 || y == 0 || y == sizeY-1) {
              class_value = 0; // Represents an obstacle.
            }
            
            trData[y][x] = class_value;
        }
    }
    
    for(int i=0; i< numTerrainClasses; ++i) {
        TraversabilityClass klass(1 - (1.0 / numTerrainClasses * i));
        trGrid->setTraversabilityClass(i, klass);
    }

    envfoo.attachItem(trGrid);
    
}

/** Changes the traversability classes of 'width' cells. */
void randUpdateMap(DStarLite& dstar_lite) {
    int cell_x = 0, cell_y = 0;
    int new_class = 0;
    for(unsigned int i=0; i<sizeX; i++) {
        cell_x = rand()%sizeX;
        cell_y = rand()%sizeY;
        new_class = rand()%numTerrainClasses;
        dstar_lite.updateTraversability(cell_x, cell_y, new_class);
    }
};

timeval start, end;

void executePlanning(DStarLite& dstar_lite, std::string test_description) {
    double path_cost = 0;
    int start_x, start_y, goal_x, goal_y;
    printf("%d map updates will be executed after each new start position\n", (int)sizeX);
    gettimeofday(&start, 0);    
    for(int i_goal=0; i_goal<num_change_goal; ++i_goal) {
        goal_x = rand()%sizeX;
        goal_y = rand()%sizeY;
        for(int i_start=0; i_start<num_change_start; ++i_start) {   
            start_x = rand()%sizeX;
            start_y = rand()%sizeY;
            randUpdateMap(dstar_lite);
            BOOST_REQUIRE_NO_THROW(path_cost = dstar_lite.run(goal_x, goal_y, start_x, start_y));
            printf("Path from start (%d,%d) to goal (%d,%d) cost: %4.2f sec.\n", start_x, start_y, goal_x, goal_y, path_cost);  
        }
    }
    gettimeofday(&end, 0);
    double time_passed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_usec - start.tv_usec) / 1000;
    printf("%s took %4.2f ms\n", test_description.c_str(), time_passed_ms);
}



BOOST_AUTO_TEST_CASE( dstar_lite_planner_norobotsize_noinflatemax )
{
    // IOnly single cells are updated (not affecting the surrounding cells).
    
    DStarLite dstar_lite;
    executePlanning(dstar_lite, "norobotsize_noinflatemax");   
}

BOOST_AUTO_TEST_CASE( dstar_lite_planner_robotsize_noinflatemax )
{
    // Uses the average cost within the footprint-/robot-size for updates.
    DStarLite dstar_lite;
    executePlanning(dstar_lite, "robotsize_noinflatemax");
}

BOOST_AUTO_TEST_CASE( dstar_lite_planner_robotsize_inflatemax )
{
    // Uses the maximal cost within the footprint-/robot-size for updates.
    DStarLite dstar_lite;
    executePlanning(dstar_lite, "robotsize_inflatemax");
}



