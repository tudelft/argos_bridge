/*
 * argos_ros_start_simulation.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: knmcguire
 */



#include "ros/ros.h"
#include "argos_bridge/argos_ros_start_sim.h"
#include "argos_bridge/argos_ros_stop_sim.h"
#include "argos_bridge/argos_ros_reset_sim.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>


// Start the ARGoS Simulator
bool start_sim(argos_bridge::argos_ros_start_sim::Request  &req,
		argos_bridge::argos_ros_start_sim::Response &res)
{


	if(req.start_argos){
		argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
		cSimulator.SetExperimentFileName("argos_worlds/bug.argos");
		cSimulator.LoadExperiment();
		cSimulator.Reset();
		cSimulator.Execute();
		res.argos_has_started = true;}


	return true;
}

// Reset the ARGoS Simulator
bool reset_sim(argos_bridge::argos_ros_reset_sim::Request  &req,
		argos_bridge::argos_ros_reset_sim::Response &res)
{

	if(req.reset_argos){
		argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();;
		cSimulator.Reset();
		res.argos_has_resetted = true;
	}

	return true;
}


// Stop the ARGoS Simulator
bool stop_sim(argos_bridge::argos_ros_stop_sim::Request  &req,
		argos_bridge::argos_ros_stop_sim::Response &res)
{


	if(req.stop_argos){
		argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();;
		cSimulator.Destroy();


		res.argos_has_stopped = true;
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "argos_ros_start_sim");
	ros::NodeHandle n;

	ros::ServiceServer service1 = n.advertiseService("argos_ros_start_sim", start_sim);
	ros::ServiceServer service2 = n.advertiseService("argos_ros_reset_sim", reset_sim);
	ros::ServiceServer service3 = n.advertiseService("argos_ros_stop_sim", stop_sim);

	ROS_INFO("Ready start or stop the ARGoS simulator");


	ros::spin();

	return 0;
}
