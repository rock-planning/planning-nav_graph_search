rock_library(nav_graph_search
    SOURCES .cpp)



rock_executable(nav_graph_search_bin main.cpp
    DEPS nav_graph_search)

