/*
 * fitness_score_loop_function.cpp
 *
 *  Large amount of content is copied from coverage_loop_functions from the repository jamesbut/Coverage_crazyflie
 *
 *  Created on: Nov 10, 2017
 *      Author: knmcguire
 */

#include "fitness_score_loop_function.h"



// Copied from argos_ros_bot.cpp
// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* FitnessScoreLoopFunction::nodeHandle = initROS();


FitnessScoreLoopFunction::FitnessScoreLoopFunction() :
		    distance(0.), position_bots(2), MAX_RANGE(14.2),
		    no_son_of_mine(false), CLOSE_TOO_TOWER(0.32) /*CLOSE_TOO_TOWER(1.0)*/ {
}
FitnessScoreLoopFunction::~FitnessScoreLoopFunction(){
}

/*Init: Get all the footbot entities and initialize distance for fitness score
 *
 */
void FitnessScoreLoopFunction::Init(TConfigurationNode& t_node)
{
  ros::NodeHandle n;

  send_end_of_sim_pub = n.advertise<std_msgs::Empty>("finished_sim_matlab", 1000);

  distance= 0.0f;
  no_son_of_mine =  false;


}

/****************************************/
//COPIED FROM space.cpp
/****************************************/

static CEmbodiedEntity* GetEmbodiedEntity(CEntity* pc_entity) {
   /* Is the entity embodied itself? */
   CEmbodiedEntity* pcEmbodiedTest = dynamic_cast<CEmbodiedEntity*>(pc_entity);
   if(pcEmbodiedTest != NULL) {
      return pcEmbodiedTest;
   }
   /* Is the entity composable with an embodied component? */
   CComposableEntity* pcComposableTest = dynamic_cast<CComposableEntity*>(pc_entity);
   if(pcComposableTest != NULL) {
      if(pcComposableTest->HasComponent("body")) {
         return &(pcComposableTest->GetComponent<CEmbodiedEntity>("body"));
      }
   }
   /* No embodied entity found */
   return NULL;
}


/*PreStep: Execute a function at every step of the simulation
 *
 * TODO: Calculate this only for an tower entity (make a new robot)
 *
 */
void FitnessScoreLoopFunction::PreStep()
{
  /* Get the map of all foot-bots from the space */
  CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
  /* Go through them */
  int id_it = 0;
  for(CSpace::TMapPerType::iterator it = tFBMap.begin();
      it != tFBMap.end();
      ++it) {
     /* Create a pointer to the current foot-bot */
     CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
     struct position_bot_t position_bot;
     position_bot.position =pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
     position_bot.bot_id_number = id_it;
     position_bots.push_back(position_bot);
     id_it ++;

     CEmbodiedEntity*  embEntity = GetEmbodiedEntity(pcFB);
     if(pcFB->GetId()=="bot0"&&embEntity->IsCollidingWithSomething()&&no_son_of_mine==false)
     {
       no_son_of_mine = true;
     }
  }

  calculateBotDistances();
  position_bots.clear();

  distances.push_back(distance);
  //std::cout << distances.size() << std::endl;
  //Check to see whether the robot is close enough to the tower to terminate
  if(distance < CLOSE_TOO_TOWER) {

     std::cout << "Close!" << std::endl;

     ros::NodeHandle n;

   //   ros::ServiceClient client_run = n.serviceClient<std_srvs::Empty>("/stop_run");
   //   std_srvs::Empty stop_run_srv;
     //
   //   if (!client_run.call(stop_run_srv)) {
   //      ROS_ERROR("Failed to tell run about stopping");
   //      exit(0);
   //   }

     ros::ServiceClient client_sim = n.serviceClient<std_srvs::Empty>("/stop_sim");
     std_srvs::Empty stop_sim_srv;

     if (!client_sim.call(stop_sim_srv)) {
        ROS_ERROR("Failed to stop sim");
        exit(0);
     }

 }

}

/*PreStep: After the simulation, send the biggest distance between the footbots
 *
 */
void FitnessScoreLoopFunction::PostExperiment()
{

  /* Get the map of all foot-bots from the space */
  CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
  /* Go through them */
  int id_it = 0;
  for(CSpace::TMapPerType::iterator it = tFBMap.begin();
      it != tFBMap.end();
      ++it) {
     /* Create a pointer to the current foot-bot */
     CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
     struct position_bot_t position_bot;
     position_bot.position =pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
     position_bot.bot_id_number = id_it;
     /* Add the current position of the foot-bot if it's sufficiently far from the last */
     position_bots.push_back(position_bot);
     id_it ++;
     }


  calculateBotDistances();
  position_bots.clear();

  std::ofstream file_fitness, file_distance;
  file_fitness.open ("fitness.txt");
  file_fitness << distance << ", " <<no_son_of_mine<<"\n";
  file_fitness.close();

  file_distance.open("distances.txt");
  for(int it = 0;it<distances.size();it++)
    file_distance << distances.at(it)<<"\n";
  file_distance.close();

  double fitness_score = MAX_RANGE - distance;
  if(no_son_of_mine)
  {
    fitness_score = fitness_score/10;
  }

  global_fitness_variable = fitness_score;

  std::cout << fitness_score << std::endl;
  std_msgs::Empty empty_msg;
  send_end_of_sim_pub.publish(empty_msg);

}

/*Reset: reinitialize fitnesscore
 *
 */
void FitnessScoreLoopFunction::Reset(){
  distance= 0.0f;
  distances.clear();
  no_son_of_mine =  false;

}

/*calculateBotDistances: calculateBotDistances and saves the largest one
 *
 */
void FitnessScoreLoopFunction::calculateBotDistances() {
  distance = 0.0f;
  for(int i = 0; i < position_bots.size(); i++) {

      for(int j = (i + 1); j < position_bots.size(); j++) {

	  float dist = Distance(position_bots[i].position,
			       position_bots[j].position);
	  if(dist > distance) distance = dist;
      }
  }
}
