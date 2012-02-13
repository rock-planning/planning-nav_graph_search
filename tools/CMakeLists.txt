find_package(Boost COMPONENTS filesystem system)
rock_executable(make_band_from_classes make_band_from_classes.cpp
    DEPS nav_graph_search
    DEPS_PLAIN Boost)
