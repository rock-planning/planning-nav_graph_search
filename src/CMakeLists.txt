rock_library(nav_graph_search
    SOURCES dstar.cpp grid_graph.cpp make_band_from_classes.cpp
        make_traversability.cpp skeleton.cpp terrain_classes.cpp 
        traversability_classifier.cpp traversability_map.cpp
    DEPS_PKGCONFIG envire
    HEADERS 
        dstar.hpp geometry.hpp grid_graph.hpp
        grid_map.hpp point.hpp pool_allocator.hpp
        skeleton.hpp terrain_classes.hpp traversability_classifier.hpp
        traversability_map.hpp
    DEPS_CMAKE     GDAL)

