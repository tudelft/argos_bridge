add_library(argos_ros_bot MODULE argos_ros_bot.h argos_ros_bot.cpp)
target_link_libraries(argos_ros_bot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(argos_ros_bot_neat MODULE argos_ros_bot_NEAT.h argos_ros_bot_NEAT.cpp)
target_link_libraries(argos_ros_bot_neat
  neatLibRos2
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(footbot_manualcontrol SHARED footbot_manualcontrol.h footbot_manualcontrol.cpp)
target_link_libraries(footbot_manualcontrol
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
  
add_library(argos_ros_bot_wf MODULE argos_ros_bot_WF.h argos_ros_bot_WF.cpp)
target_link_libraries(argos_ros_bot_wf
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)
  
  add_library(argos_ros_bot_wf_neat MODULE argos_ros_bot_WF_NEAT.h argos_ros_bot_WF_NEAT.cpp)
target_link_libraries(argos_ros_bot_wf_neat
  neatLibRos2
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)