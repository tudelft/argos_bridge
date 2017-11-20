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
		    distance(0.), position_bots(2), MAX_RANGE(14.2) {
}
FitnessScoreLoopFunction::~FitnessScoreLoopFunction(){
}

/*Init: Get all the footbot entities and initialize distance for fitness score
 *
 */
void FitnessScoreLoopFunction::Init(TConfigurationNode& t_node)
{

  distance= 0.0f;


}



/*PreStep: Execute a function at every step of the simulation
 *
 * TODO: Calculate this only for an tower entity (make a new robot)
 *
 */
void FitnessScoreLoopFunction::PreStep()
{

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


  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<neat_ros::FinishedSim>("finished_sim");
  neat_ros::FinishedSim service_msg;
  service_msg.request.fitness_score = MAX_RANGE - distance;
  client.call(service_msg);
  //std::cout<<"service has been send with "<<MAX_RANGE - distance <<std::endl;
}

/*Reset: reinitialize fitnesscore
 *
 */
void FitnessScoreLoopFunction::Reset(){
  distance= 0.0f;
  position_bots.clear();
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
