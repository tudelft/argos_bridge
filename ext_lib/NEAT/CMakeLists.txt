cmake_minimum_required(VERSION 2.8.12)

PROJECT( neat )

include_directories(include)

FILE(GLOB HEADER_FILES include/*.h)
FILE(GLOB neat_sources src/*.cpp)

#ADD_EXECUTABLE( neat ${neat_sources} ${HEADER_FILES} )
add_library(neatLibRos2 SHARED ${neat_sources} ${HEADER_FILES})
