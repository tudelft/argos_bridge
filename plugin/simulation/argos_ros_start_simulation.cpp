/*
 * argos_ros_start_simulation.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: knmcguire
 */



#include "ros/ros.h"
#include "argos_bridge/argos_ros_start_sim.h"
#include "std_srvs/Empty.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>
#include <boost/thread.hpp>

bool start_sim_bool = false;

// Start the ARGoS Simulator via callaback
bool start_sim(std_srvs::Empty::Request  &req,
					std_srvs::Empty::Response &res)
{

	start_sim_bool = true;

	return true;
}

//Thread to listen for start sim service
void startSimServiceThread() {

	ros::NodeHandle n;
	ros::ServiceServer service1 = n.advertiseService("start_sim", &start_sim);
	ros::spin();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "argos_ros_start_sim");

	//Start listening for start_sim service
	boost::thread spin_thread(&startSimServiceThread);

	//TODO: Make filename part of launch file
	argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
	cSimulator.SetExperimentFileName("/home/james/catkin_ws/src/argos_bridge/argos_worlds/bug.argos");
	//cSimulator.SetExperimentFileName("/home/knmcguire/Documents/Software/catkin_ws/src/argos_bridge/argos_worlds/bug.argos");

	cSimulator.LoadExperiment();

	while(true) {

		//Only execute when start_sim is received from service
		while(!start_sim_bool) {}

		cSimulator.Reset();
		cSimulator.Execute();

		start_sim_bool = false;

	}


	return 0;
}
