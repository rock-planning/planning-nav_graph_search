set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}-Wno-deprecated)

rock_library(nav_graph_search
    SOURCES dstar.cpp grid_graph.cpp search.cpp
        terrain_classes.cpp 
        traversability_map.cpp
        dstar_lite.cpp dstar_lite/DStarLite.cpp
    DEPS_PKGCONFIG envire base-lib
    HEADERS 
        dstar.hpp geometry.hpp grid_graph.hpp search.hpp
        grid_map.hpp point.hpp pool_allocator.hpp
        terrain_classes.hpp
        traversability_map.hpp
        dstar_lite.hpp dstar_lite/DStarLite.h
    DEPS_CMAKE     GDAL
)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
pkg_check_modules(GL REQUIRED gl)
if (OPENGL_FOUND AND GLUT_FOUND AND GL_FOUND)
message("-- OpenGL, GLUT and GL found, dstar-lite test program will be created")
rock_executable(dstar_lite_gui
    SOURCES dstar_lite/DStarLiteDraw.cpp 
        dstar_lite/DStarLite.cpp
    HEADERS
        dstar_lite/DStarLite.h
    DEPS_CMAKE GLUT OpenGL
    DEPS_PKGCONFIG gl base-lib
)
set_target_properties(dstar_lite_gui PROPERTIES COMPILE_FLAGS "-DUSE_OPEN_GL")
endif (OPENGL_FOUND AND GLUT_FOUND AND GL_FOUND) 

