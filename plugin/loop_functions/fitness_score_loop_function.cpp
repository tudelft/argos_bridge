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
		    distance(0.), position_bots(2), MAX_RANGE(28.5) {
}
FitnessScoreLoopFunction::~FitnessScoreLoopFunction(){
}

/*Init: Get all the footbot entities and initialize distance for fitness score
 *
 */
void FitnessScoreLoopFunction::Init(TConfigurationNode& t_node)
{
  // Create the subscribers
  // TODO make this dependable on a global num_bot ?
  std::string botPoseTopic1 ="/bot0/position";
  std::string botPoseTopic2 ="/bot1/position";

  if(ros::ok())
    {
      ros::NodeHandle n;
      botPoseSub1 = n.subscribe(botPoseTopic1, 1000, & FitnessScoreLoopFunction::botPoseCallback, this);
      botPoseSub2 = n.subscribe(botPoseTopic2, 1000, & FitnessScoreLoopFunction::botPoseCallback, this);
    }


  distance= 0.0f;
}


/*botPoseCallback: Saves positions from bots, retrieved from rostopic
 *
 */
void FitnessScoreLoopFunction::botPoseCallback(const geometry_msgs::PoseStamped& pose_stamped)
{
  //Extract number from topic name
  int currentId;
  char c;
  std::stringstream iss2(pose_stamped.header.frame_id);
  iss2  >> c >> c >> c >> currentId;

  //Place bot's position, sorted on the bot's ID
  struct position_bot_t position_bot;
  position_bots[currentId].bot_id_number = currentId;
  position_bots[currentId].position.SetX(pose_stamped.pose.position.x);
  position_bots[currentId].position.SetY(pose_stamped.pose.position.y);
  position_bots[currentId].position.SetZ(pose_stamped.pose.position.z);
}

/*PreStep: Execute a function at every step of the simulation
 *
 * TODO: Calculate this only for an tower entity (make a new robot)
 *
 */
void FitnessScoreLoopFunction::PreStep()
{
  calculateBotDistances();
  std::cout << distance << std::endl;
}

/*PreStep: After the simulation, send the biggest distance between the footbots
 *
 */
void FitnessScoreLoopFunction::PostExperiment()
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<neat_ros::FinishedSim>("finished_sim");
  neat_ros::FinishedSim service_msg;
  service_msg.request.fitness_score = MAX_RANGE - distance;
  client.call(service_msg);
  std::cout<<"service has been send"<<std::endl;
}

/*Reset: reinitialize fitnesscore
 *
 */
void FitnessScoreLoopFunction::Reset(){

  distance= 0.0f;
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



REGISTER_LOOP_FUNCTIONS(FitnessScoreLoopFunction, "fitness_score_loop_function");
