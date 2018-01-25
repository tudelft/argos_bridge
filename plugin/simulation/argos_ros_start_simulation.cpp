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
std::string file_name_env;

extern double global_fitness_variable;

int file_name_env_number;
int trial_num;

// Start the ARGoS Simulator via callaback
bool start_sim(neat_ros::StartSim::Request  &req,
	       		neat_ros::StartSim::Response &res)
{

  regen_env = req.regenerate_env;
  //std::cout << "Regen Env: " << regen_env << std::endl;
  file_name_env_number = req.select_env;
  //std::cout << "Env Num: " << file_name_env_number << std::endl;
  //std::cout << "Trial num: " << req.trial_num << std::endl;
  trial_num = req.trial_num;
	start_sim_bool = true;
}

bool stop_sim(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Request &res)
{
  std::cout<<"received_stop_sim"<<std::endl;
  argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
  cSimulator.Terminate();
}

//Thread to listen for start sim service
void startSimServiceThread() {

  ros::NodeHandle n;
  ros::ServiceServer service1 = n.advertiseService("start_sim", &start_sim);
  ros::ServiceServer service2 = n.advertiseService("stop_sim", &stop_sim);
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

  //std::string path = ros::package::getPath("argos_bridge");
  //std::cout << path << std::endl;
  std::string path = "/home/james/catkin_ws/src/argos_bridge";

	argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
	std::string argos_world_file_name;

	if(ros::param::get("~argos_world_file_name",argos_world_file_name))
	  cSimulator.SetExperimentFileName(path + argos_world_file_name);
	else
	  cSimulator.SetExperimentFileName(path + "/argos_worlds/rand_environments/one_wall.argos");
  std::cout<<"Opening ARGOS file in :"<<path + argos_world_file_name<<std::endl;

  std::string file_name_env_path_rel;
  std::string file_name_env_path;

  if(ros::param::get("~file_name_env_path_rel",file_name_env_path_rel)) {
    file_name_env_path =  path+file_name_env_path_rel;
	 //std::cout << "FILE NAME: " << file_name_env_path << std::endl;
  }else {
    file_name_env_path = path+"/argos_worlds/rand_environments/rand_env_";
  }
  std::cout<<"Opening environment_files in :"<<file_name_env_path<<std::endl;

  cSimulator.LoadExperiment();

	ros::Rate loop_rate(100);

  	//Note to self, ros::ok() is a must for while loop in ROS!
   while(ros::ok()) {
			//std::cout << "Running" << std::endl;
   	//Only execute when start_sim is received from service
  		if(start_sim_bool) {

  		  if (regen_env==3)
  		  {
  		    file_name_env = file_name_env_path + std::to_string(file_name_env_number) + ".png";
  		    //std::cout<<"New environment generated with "<< file_name_env<<std::endl;
  		  }
  			//std::cout << "Resetting sim.." <<std::endl;			//These are here to debug why it sometimes sticks
       	cSimulator.Reset();
  			//std::cout << "..Sim resetted" << std::endl;
  			//std::cout << "Start sim.." << std::endl;
       	cSimulator.Execute();
  			//std::cout << "..End sim" << std::endl;
       	start_sim_bool = false;
			  sendFinishedService();

  		}

		loop_rate.sleep();

  }

    cSimulator.Terminate();

  spin_thread.join();

  return 0;
}
