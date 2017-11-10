/*
 * argos_ros_start_simulation.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: knmcguire
 */

#include "argos_ros_start_simulation.h"

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
  //cSimulator.SetExperimentFileName("/home/james/catkin_ws/src/argos_bridge/argos_worlds/bug.argos");
  cSimulator.SetExperimentFileName("/home/knmcguire/Documents/Software/catkin_ws/src/argos_bridge/argos_worlds/bug.argos");

  cSimulator.LoadExperiment();

  //Note to self, ros::ok() is a must for while loop in ROS!
  while(ros::ok()) {

      //Only execute when start_sim is received from service
      if(!start_sim_bool)
	{

	}else
	  {
	    cSimulator.Reset();
	    cSimulator.Execute();
	    start_sim_bool = false;
	  }
  }

  return 0;
}
