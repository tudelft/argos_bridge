
include_directories(${CMAKE_SOURCE_DIR})

add_library(argos_ros_start_simulation SHARED argos_ros_start_simulation.cpp argos_ros_start_simulation.h)

target_link_libraries(argos_ros_start_simulation
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
  fitness_score)
