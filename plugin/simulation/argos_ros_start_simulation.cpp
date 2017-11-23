#/*
 * argos_ros_start_simulation.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: knmcguire
 */

#include "argos_ros_start_simulation.h"

bool start_sim_bool = false;
int regen_env;


// Start the ARGoS Simulator via callaback
bool start_sim(neat_ros::StartSim::Request  &req,
	       		neat_ros::StartSim::Response &res)
{
  start_sim_bool = true;
  regen_env = req.regenerate_env;
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


  std::string path = ros::package::getPath("argos_bridge");


	argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
  std::cout<<"Opening ARGOS file in :"<<path<<"/argos_worlds/rand_env_test.argos"<<std::endl;
	cSimulator.SetExperimentFileName(path + "/argos_worlds/rand_env_test.argos");

  	cSimulator.LoadExperiment();

  	//Note to self, ros::ok() is a must for while loop in ROS!
  	while(ros::ok()) {
   	//Only execute when start_sim is received from service
		if(start_sim_bool) {
			std::cout << "Resetting.." <<std::endl;
	      cSimulator.Reset();
			std::cout << "..Resetted" << std::endl;
			std::cout << "Start sim.." << std::endl;
	      cSimulator.Execute();
			std::cout << "..End sim" << std::endl;
	      start_sim_bool = false;

	 	}

  }

  spin_thread.join();

  return 0;
}
