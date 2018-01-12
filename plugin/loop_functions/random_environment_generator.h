/*
 * random_environment_generator.h
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#ifndef ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_
#define ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_

//ARGoS libraries
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/entity.h>




//OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Standard C++ libraries
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>    // std::rotate
#include <vector>       // std::vector
#include <stdlib.h>     /* srand, rand */
#include <stdio.h>
#include <ctime>
#include <chrono>

using namespace argos;

struct grid_element_status_t {
  bool is_agent_present;
  std::vector<std::vector<int>> circ_action;
  bool is_corridor_present;
};

class RandomEnvironmentGenerator
{
public:
  RandomEnvironmentGenerator();
  virtual void Init(TConfigurationNode &t_node);
  virtual void Reset(std::string file_name);
  virtual void Destroy();

  void initializeGrid();
  void initializeAgents();
  void findAgents();
  void decideNextAction(std::vector<int> current_bot_position);
  void setNextLocation(std::vector<int> current_bot_position);
  float getCorridorPercentage();
  void makeBinaryImageCorridors();
  void checkConnectivity();
  void checkConnectivityOpenCV();

  void makeBoundariesCorridors();
  void makeRooms();
  void makeRandomOpenings();
  void putBlocksInEnvironment();
  void putLinesInEnvironment();
  void generateEnvironment();
  void generateEnvironmentFromFile(std::string file_name);
  void getRobotPositions();
  void ClearEnvironment();
  void dfs(int x, int y, int current_label);


private:
  std::vector<std::vector<grid_element_status_t>> environment_grid;
  int environment_width;
  int environment_height;
  std::vector<std::vector<int>> initial_bot_positions;
  std::vector<std::vector<int>> current_agent_positions;
  float change_agent_gostraight;
  float wanted_corridor_percentage;
  cv::Mat bin_corridor_img;
  cv::Mat bin_corridor_img_large;
  cv::Mat corridor_contours_img;
  cv::Mat corridor_contours_img_save;

  float room_percentage;
  int total_boxes_generated;
  std::vector<CBoxEntity*> boxEntities;
  int amount_of_openings;
  bool environment_accepted;
  cv::RNG rng;
  std::vector<std::vector<int>> connectivity_labels;

  int it_box;

  bool corridors_are_connected;



};


#endif /* ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_ */
