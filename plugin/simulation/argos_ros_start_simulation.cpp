#/*
 * argos_ros_start_simulation.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: knmcguire
 */

#include "argos_ros_start_simulation.h"
#include "neat_ros/FinishedSim.h"

#include "../loop_functions/fitness_score_loop_function.h"

bool start_sim_bool = false;
int regen_env;

extern double global_fitness_variable;

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

void sendFinishedService() {

	ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<neat_ros::FinishedSim>("finished_sim");
   neat_ros::FinishedSim service_msg;
   service_msg.request.fitness_score = global_fitness_variable;
	//std::cout << global_fitness_variable << std::endl;
   client.call(service_msg);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "argos_ros_start_sim");

	//Start listening for start_sim service
	boost::thread spin_thread(&startSimServiceThread);


  std::string path = ros::package::getPath("argos_bridge");

	argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
  //std::cout<<"Opening ARGOS file in :"<<path<<"/argos_worlds/rand_environments/rand_env_6.argos"<<std::endl;
	//cSimulator.SetExperimentFileName(path + "/argos_worlds/rand_environments/no_walls.argos");
	cSimulator.SetExperimentFileName(path + "/argos_worlds/rand_environments/two_walls.argos");
	//cSimulator.SetExperimentFileName(path + "/argos_worlds/rand_environments/two_walls_mod.argos");

  	cSimulator.LoadExperiment();
	//start_sim_bool = true;

	ros::Rate loop_rate(100);

  	//Note to self, ros::ok() is a must for while loop in ROS!
   while(ros::ok()) {
			//std::cout << "Running" << std::endl;
   	//Only execute when start_sim is received from service
  		if(start_sim_bool) {
  			std::cout << "Resetting sim.." <<std::endl;			//These are here to debug why it sometimes sticks
       	cSimulator.Reset();
  			std::cout << "..Sim resetted" << std::endl;
  			std::cout << "Start sim.." << std::endl;
       	cSimulator.Execute();
  			std::cout << "..End sim" << std::endl;
       	start_sim_bool = false;
			sendFinishedService();

  		}

		loop_rate.sleep();

  }

  spin_thread.join();

  return 0;
}
