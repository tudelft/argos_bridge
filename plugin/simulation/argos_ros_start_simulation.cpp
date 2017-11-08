/*
 * argos_ros_start_simulation.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: knmcguire
 */



#include "ros/ros.h"
#include "argos_bridge/argos_ros_start_sim.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>


// Start the ARGoS Simulator
bool start_sim(argos_bridge::argos_ros_start_sim::Request  &req,
		argos_bridge::argos_ros_start_sim::Response &res)
{
	argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
	cSimulator.Reset();
	cSimulator.Execute();
	std::cout<<"ARGoS is finished"<<std::endl;


	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "argos_ros_start_sim");
	ros::NodeHandle n;

	ros::ServiceServer service1 = n.advertiseService("argos_ros_start_sim", start_sim);

	ROS_INFO("Ready start or stop the ARGoS simulator");

	//TODO: Make filename part of launch file
	argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
	cSimulator.SetExperimentFileName("/home/knmcguire/Documents/Software/catkin_ws/src/argos_bridge/argos_worlds/bug.argos");

	cSimulator.LoadExperiment();

	ros::spin();

	return 0;
}
