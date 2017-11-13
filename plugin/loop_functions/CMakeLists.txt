include_directories(${CMAKE_SOURCE_DIR})

add_library(idloop SHARED id_qtuser_functions.cpp id_qtuser_functions.h)

target_link_libraries(idloop
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_footbot
)


add_library(fitness_score SHARED fitness_score_loop_function.cpp fitness_score_loop_function.h)

target_link_libraries(fitness_score
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_footbot
)


add_library(random_environment SHARED random_environment_generator.cpp random_environment_generator.h)
find_package( OpenCV REQUIRED )
target_link_libraries(random_environment
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_footbot
  ${OpenCV_LIBS} 
)

if(ARGOS_COMPILE_QTOPENGL)
   target_link_libraries(fitness_score
   argos3plugin_simulator_qtopengl
)
endif(ARGOS_COMPILE_QTOPENGL)

find_package(OpenGL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)



