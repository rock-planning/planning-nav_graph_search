rock_library(nav_graph_search
    SOURCES dummy.cpp)

rock_executable(nav_graph_search_bin main.cpp
    DEPS nav_graph_search)

