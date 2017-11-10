/*
 * argos_ros_start_simulation.h
 *
 *  Created on: Nov 11, 2017
 *      Author: knmcguire
 */

#ifndef ARGOS_BRIDGE_PLUGIN_SIMULATION_ARGOS_ROS_START_SIMULATION_H_
#define ARGOS_BRIDGE_PLUGIN_SIMULATION_ARGOS_ROS_START_SIMULATION_H_

//ROS include files
#include "ros/ros.h"
#include "std_srvs/Empty.h"

//ARGoS include files
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>


//Standard C++ libraries
#include <iostream>
#include <sstream>
#include <string>
#include <boost/thread.hpp>

using namespace argos;


void startSimServiceThread();
bool start_sim(std_srvs::Empty::Request  &req,
	       std_srvs::Empty::Response &res);


#endif /* ARGOS_BRIDGE_PLUGIN_SIMULATION_ARGOS_ROS_START_SIMULATION_H_ */
