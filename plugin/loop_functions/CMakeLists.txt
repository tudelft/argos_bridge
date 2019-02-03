include_directories(${CMAKE_SOURCE_DIR})

include(ARGoSCheckQTOpenGL)

if(ARGOS_COMPILE_QTOPENGL)

   add_library(idloop SHARED id_qtuser_functions.cpp id_qtuser_functions.h)

   target_link_libraries(idloop
      argos3core_simulator
      argos3plugin_simulator_entities
      argos3plugin_simulator_media
      argos3plugin_simulator_footbot
   )

endif(ARGOS_COMPILE_QTOPENGL)


add_library(fitness_score SHARED fitness_score_loop_function.cpp fitness_score_loop_function.h)

target_link_libraries(fitness_score
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_footbot
)

add_library(trajectory_loop_functions SHARED 
  trajectory_loop_functions.h
  trajectory_qtuser_functions.h
  trajectory_loop_functions.cpp
  trajectory_qtuser_functions.cpp)

target_link_libraries(trajectory_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})

find_package( OpenCV REQUIRED )
message(STATUS "OpenCV Libraries: " ${OpenCV_LIBS})
if(OpenCV_FOUND)
add_library(random_environment SHARED random_environment_generator.cpp random_environment_generator.h)
find_package( OpenCV REQUIRED )
target_link_libraries(random_environment
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_footbot
  ${OpenCV_LIBS} 
)

add_library(master_loop SHARED master_loop_function.cpp master_loop_function.h)

target_link_libraries(master_loop
  random_environment
  fitness_score
  trajectory_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_footbot
  ${OpenCV_LIBS} 
)

add_library(libid_loop_functions SHARED id_qtuser_functions.cpp id_qtuser_functions.h)

target_link_libraries(libid_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${OpenCV_LIBS} 
)


endif(OpenCV_FOUND)

#if(ARGOS_COMPILE_QTOPENGL)
#   target_link_libraries(fitness_score
#   argos3plugin_simulator_qtopengl
#)
#endif(ARGOS_COMPILE_QTOPENGL)

#find_package(OpenGL REQUIRED)
#FIND_PACKAGE(Qt4 REQUIRED)



