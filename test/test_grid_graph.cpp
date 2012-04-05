#define BOOST_TEST_MODULE NavGraphSearch 
#include <boost/test/included/unit_test.hpp>

#include <nav_graph_search/grid_graph.hpp>

using namespace nav_graph_search;

BOOST_AUTO_TEST_CASE( test_neighbour_iterator_normal_iteration )
{
    GridGraph graph(100, 100);
    for (int y = 0; y < 100; ++y)
        for (int x = 0; x < 100; ++x)
            graph.setValue(x, y, x * 100 + y);

    NeighbourConstIterator it = graph.neighboursBegin(50, 60);

    BOOST_CHECK_CLOSE(5060, it.getSourceValue(), 1e-6);
    BOOST_CHECK_EQUAL(50, it.sourceX());
    BOOST_CHECK_EQUAL(60, it.sourceY());

    BOOST_CHECK_EQUAL(GridGraph::RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(51, it.x());
    BOOST_CHECK_EQUAL(60, it.y());
    BOOST_CHECK_CLOSE(5160, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::TOP_RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(51, it.x());
    BOOST_CHECK_EQUAL(61, it.y());
    BOOST_CHECK_CLOSE(5161, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::TOP, it.getNeighbour());
    BOOST_CHECK_EQUAL(50, it.x());
    BOOST_CHECK_EQUAL(61, it.y());
    BOOST_CHECK_CLOSE(5061, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::TOP_LEFT, it.getNeighbour());
    BOOST_CHECK_EQUAL(49, it.x());
    BOOST_CHECK_EQUAL(61, it.y());
    BOOST_CHECK_CLOSE(4961, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::LEFT, it.getNeighbour());
    BOOST_CHECK_EQUAL(49, it.x());
    BOOST_CHECK_EQUAL(60, it.y());
    BOOST_CHECK_CLOSE(4960, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::BOTTOM_LEFT, it.getNeighbour());
    BOOST_CHECK_EQUAL(49, it.x());
    BOOST_CHECK_EQUAL(59, it.y());
    BOOST_CHECK_CLOSE(4959, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::BOTTOM, it.getNeighbour());
    BOOST_CHECK_EQUAL(50, it.x());
    BOOST_CHECK_EQUAL(59, it.y());
    BOOST_CHECK_CLOSE(5059, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::BOTTOM_RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(51, it.x());
    BOOST_CHECK_EQUAL(59, it.y());
    BOOST_CHECK_CLOSE(5159, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK(it.isEnd());

    ++it;
    BOOST_CHECK(it.isEnd());
}

BOOST_AUTO_TEST_CASE( test_neighbour_iterator_partial_iteration )
{
    GridGraph graph(100, 100);
    for (int x = 0; x < 100; ++x)
        for (int y = 0; y < 100; ++y)
            graph.setValue(x, y, x * 100 + y);

    NeighbourConstIterator it = graph.neighboursBegin(50, 60, GridGraph::TOP | GridGraph::BOTTOM);

    BOOST_CHECK_CLOSE(5060, it.getSourceValue(), 1e-6);
    BOOST_CHECK_EQUAL(50, it.sourceX());
    BOOST_CHECK_EQUAL(60, it.sourceY());

    BOOST_CHECK_EQUAL(GridGraph::TOP, it.getNeighbour());
    BOOST_CHECK_EQUAL(50, it.x());
    BOOST_CHECK_EQUAL(61, it.y());
    BOOST_CHECK_CLOSE(5061, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK_EQUAL(GridGraph::BOTTOM, it.getNeighbour());
    BOOST_CHECK_EQUAL(50, it.x());
    BOOST_CHECK_EQUAL(59, it.y());
    BOOST_CHECK_CLOSE(5059, it.getTargetValue(), 1e-6);

    ++it;
    BOOST_CHECK(it.isEnd());

    ++it;
    BOOST_CHECK(it.isEnd());
}

BOOST_AUTO_TEST_CASE( test_neighbour_iterator_mask_on_border)
{
    GridGraph graph(100, 100);

    int NOT_TOP = ~(GridGraph::TOP | GridGraph::TOP_LEFT | GridGraph::TOP_RIGHT);
    int NOT_BOTTOM = ~(GridGraph::BOTTOM | GridGraph::BOTTOM_LEFT | GridGraph::BOTTOM_RIGHT);
    int NOT_LEFT = ~(GridGraph::LEFT | GridGraph::BOTTOM_LEFT | GridGraph::TOP_LEFT);
    int NOT_RIGHT = ~(GridGraph::RIGHT | GridGraph::BOTTOM_RIGHT | GridGraph::TOP_RIGHT);

    NeighbourConstIterator it;

    it = graph.neighboursBegin(0, 10);
    BOOST_CHECK_EQUAL(GridGraph::RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(GridGraph::DIR_ALL & NOT_LEFT, it.getMask());

    it = graph.neighboursBegin(99, 10);
    BOOST_CHECK_EQUAL(GridGraph::TOP, it.getNeighbour());
    BOOST_CHECK_EQUAL((GridGraph::DIR_ALL & NOT_RIGHT) >> 2, it.getMask());

    it = graph.neighboursBegin(10, 0);
    BOOST_CHECK_EQUAL(GridGraph::RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(GridGraph::DIR_ALL & NOT_BOTTOM, it.getMask());

    it = graph.neighboursBegin(10, 99);
    BOOST_CHECK_EQUAL(GridGraph::RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(GridGraph::DIR_ALL & NOT_TOP, it.getMask());


    it = graph.neighboursBegin(0, 0);
    BOOST_CHECK_EQUAL(GridGraph::RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(GridGraph::DIR_ALL & NOT_BOTTOM & NOT_LEFT, it.getMask());

    it = graph.neighboursBegin(99, 0);
    BOOST_CHECK_EQUAL(GridGraph::TOP, it.getNeighbour());
    BOOST_CHECK_EQUAL((GridGraph::DIR_ALL & NOT_RIGHT & NOT_BOTTOM) >> 2, it.getMask());

    it = graph.neighboursBegin(0, 99);
    BOOST_CHECK_EQUAL(GridGraph::RIGHT, it.getNeighbour());
    BOOST_CHECK_EQUAL(GridGraph::DIR_ALL & NOT_LEFT & NOT_TOP, it.getMask());

    it = graph.neighboursBegin(99, 99);
    BOOST_CHECK_EQUAL(GridGraph::LEFT, it.getNeighbour());
    BOOST_CHECK_EQUAL((GridGraph::DIR_ALL & NOT_RIGHT & NOT_TOP) >> 4, it.getMask());
}

