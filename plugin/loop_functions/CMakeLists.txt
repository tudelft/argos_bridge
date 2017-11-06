include_directories(${CMAKE_SOURCE_DIR})

add_library(idloop SHARED id_qtuser_functions.cpp id_qtuser_functions.h)

find_package(OpenGL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)



