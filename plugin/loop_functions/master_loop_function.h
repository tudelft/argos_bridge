/*
 * fitness_score_loop_function.h
 *
 *  Created on: Nov 10, 2017
 *      Author: knmcguire
 */

#ifndef ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_MASTER_LOOP_FUNCTION_H_
#define ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_MASTER_LOOP_FUNCTION_H_



#include "ros/ros.h"


//ARGoS libraries
#include <argos3/core/simulator/loop_functions.h>

//Other Loopfunctions
#include "fitness_score_loop_function.h"
#include "random_environment_generator.h"

//Standard C++ libraries
#include <iostream>
#include <sstream>
#include <string>

using namespace argos;


class MasterLoopFunction : public CLoopFunctions {
public:
  MasterLoopFunction();
  virtual ~MasterLoopFunction();
  virtual void Init(TConfigurationNode& t_node);
  virtual void Reset();
  virtual void PreStep();
  virtual void PostExperiment();

  RandomEnvironmentGenerator randomEnvironmentGenerator;
  FitnessScoreLoopFunction fitnessScoreLoopFunction;


private:
};

#endif /* ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_MASTER_LOOP_FUNCTION_H_ */
