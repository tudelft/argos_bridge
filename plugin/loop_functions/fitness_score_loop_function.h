/*
 * fitness_score_loop_function.h
 *
 *  Created on: Nov 10, 2017
 *      Author: knmcguire
 */

#ifndef ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_FITNESS_SCORE_LOOP_FUNCTION_H_
#define ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_FITNESS_SCORE_LOOP_FUNCTION_H_


//ROS Libraries and msgs
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "neat_ros/FinishedSim.h"


//ARGoS libraries
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector3.h>


//Standard C++ libraries
#include <iostream>
#include <sstream>
#include <string>

using namespace argos;

struct position_bot_t{
  int bot_id_number;
  CVector3 position;
};

class FitnessScoreLoopFunction : public CLoopFunctions {
public:
  FitnessScoreLoopFunction();
  virtual ~FitnessScoreLoopFunction();
  virtual void Init(TConfigurationNode& t_node);
  virtual void Reset();
  virtual void PreStep();
  virtual void PostExperiment();
  void botPoseCallback(const geometry_msgs::PoseStamped& pose);
  static ros::NodeHandle* nodeHandle;

private:
  float distance;
  std::vector<position_bot_t> position_bots;
  ros::Subscriber botPoseSub1;
  ros::Subscriber botPoseSub2;
  void calculateBotDistances();
  //Handle for the node

  const double MAX_RANGE;
};

#endif /* ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_FITNESS_SCORE_LOOP_FUNCTION_H_ */
