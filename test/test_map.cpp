#define BOOST_TEST_MODULE NavGraphSearch 
#include <boost/test/included/unit_test.hpp>

#include <nav_graph_search/traversability_map.hpp>
#include <envire/maps/Grids.hpp>

using namespace nav_graph_search;
using namespace envire;


BOOST_AUTO_TEST_CASE( test_traversibility_map )
{
    Environment *env = new Environment();
    FrameNode *fn = new FrameNode();
    fn->setTransform( Eigen::Affine3d( Eigen::Translation3d( 0, 1, 2 ) ) );
    env->addChild( env->getRootNode(), fn );

    // generate a grid of 10 x 10 with 0.1 spacing and an offset to be at the
    // center of the grid (5,5)
    Grid<uint8_t> *grid = new Grid<uint8_t>( 100, 100, 0.1, 0.1, -5.0, -5.0);
    grid->getGridData( TraversabilityGrid::TRAVERSABILITY );
    env->setFrameNode( grid, fn );

    TraversabilityMap *map = TraversabilityMap::load( 
	    *grid, TraversabilityGrid::TRAVERSABILITY, 
	    std::list<TerrainClass>() );

    // now make sure, that the grid to world coordinate transformation
    // give the same values for both the envire map and the other one

    const int xi = 0, yi = 0;
    double x, y;
    grid->fromGrid( xi, yi, x, y );
    // compensate for center of cell which is used in envire
    Eigen::Vector3d res1 = grid->fromMap( Eigen::Vector3d( x, y, 0 ) ) 
	- Eigen::Vector3d( 0.05, 0.05, 0 );

    Eigen::Vector3d res2 = map->toWorld( PointID(xi, yi) );

    BOOST_CHECK_EQUAL( res1.x(), res2.x() );
    BOOST_CHECK_EQUAL( res1.y(), res2.y() );
    BOOST_CHECK_EQUAL( res1.z(), res2.z() );

    delete map;
    delete env;
}
