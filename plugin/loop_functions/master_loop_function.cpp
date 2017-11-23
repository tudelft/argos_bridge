/*
 * fitness_score_loop_function.cpp
 *
 *  Large amount of content is copied from coverage_loop_functions from the repository jamesbut/Coverage_crazyflie
 *
 *  Created on: Nov 10, 2017
 *      Author: knmcguire
 */

#include "master_loop_function.h"

extern int regen_env;

#define RANDOM_ENVIRONMENT_GEN_ON false

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


MasterLoopFunction::MasterLoopFunction() {
}
MasterLoopFunction::~MasterLoopFunction(){
}

/*Init: Get all the footbot entities and initialize distance for fitness score
 *
 */
void MasterLoopFunction::Init(TConfigurationNode& t_node)
{
  fitnessScoreLoopFunction.Init(t_node);
#if(RANDOM_ENVIRONMENT_GEN_ON)
 randomEnvironmentGenerator.Init( t_node);
#endif
}


/*Reset: reinitialize fitnesscore
 *
 */
void MasterLoopFunction::Reset(){

  fitnessScoreLoopFunction.Reset();

#if(RANDOM_ENVIRONMENT_GEN_ON)
  if(regen_env==1) {
    randomEnvironmentGenerator.Reset();
  }
#endif


}


/*PreStep: Execute a function at every step of the simulation
 * *
 */
void MasterLoopFunction::PreStep()
{
  fitnessScoreLoopFunction.PreStep();
}

/*PreStep: Before the simulation,
 *
 */
void MasterLoopFunction::PostExperiment()
{
  fitnessScoreLoopFunction.PostExperiment();

}




REGISTER_LOOP_FUNCTIONS(MasterLoopFunction, "master_loop_function");
