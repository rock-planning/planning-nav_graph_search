rock_testsuite(test_suite test_grid_graph.cpp
    DEPS nav_graph_search)

rock_testsuite(test_map test_map.cpp
    DEPS nav_graph_search 
    DEPS_PKGCONFIG envire)

rock_testsuite(test_planner test_planner.cpp
    DEPS nav_graph_search)
