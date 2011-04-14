rock_library(nav_graph_search
    SOURCES dstar.cpp grid_graph.cpp
        terrain_classes.cpp 
        traversability_classifier.cpp traversability_map.cpp
    DEPS_PKGCONFIG envire base-lib
    HEADERS 
        dstar.hpp geometry.hpp grid_graph.hpp
        grid_map.hpp point.hpp pool_allocator.hpp
        terrain_classes.hpp traversability_classifier.hpp
        traversability_map.hpp
    DEPS_CMAKE     GDAL)

rock_executable(make_band_from_classes make_band_from_classes.cpp
    DEPS nav_graph_search)
    
rock_executable(make_traversability make_traversability.cpp
    DEPS nav_graph_search)
    
