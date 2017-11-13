/*
 * random_environment_generator.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#include "random_environment_generator.h"

using namespace std;
using namespace cv;


RandomEnvironmentGenerator::RandomEnvironmentGenerator() :
  environment_width(10),
  environment_height(10),
  change_agent_gostraight(0.85f),
  wanted_corridor_percentage(0.40f),
  room_percentage(0.3f) {}

void RandomEnvironmentGenerator::Init(TConfigurationNode &t_node)
{
  srand(time(NULL));

  generateEnvironment();

}

void RandomEnvironmentGenerator::Reset()
{
  CSpace::TMapPerType &tBoxMap = GetSpace().GetEntitiesByType("box");

  for (CSpace::TMapPerType::iterator it = tBoxMap.begin(); it != tBoxMap.end(); ++it) {
    CBoxEntity *pcBox = any_cast<CBoxEntity *>(it->second);
    RemoveEntity(*pcBox);
  }

  generateEnvironment();
}
void RandomEnvironmentGenerator::Destroy()
{
  destroyAllWindows();
}

void RandomEnvironmentGenerator::generateEnvironment(void)
{
  bin_corridor_img = Mat::zeros(environment_width, environment_height, CV_8UC1);
  corridors_are_connected = false;

  while (!corridors_are_connected) {
    initializeGrid();
    initializeAgents();
    for (int it_total = 0; it_total < 100; it_total++) {

      findAgents();

      for (int it = 0; it < current_agent_positions.size(); it++) {
        decideNextAction(current_agent_positions.at(it));
        setNextLocation(current_agent_positions.at(it));

      }
      if (getCorridorPercentage() > wanted_corridor_percentage) {
        break;
      }
      makeBinaryImageCorridors();
    }
    checkConnectivity();
  }

  makeBoundariesCorridors();
  makeRooms();
  makeRandomOpenings();
  cv::Rect border(cv::Point(0, 0), corridor_contours_img.size());
  rectangle(corridor_contours_img, border, Scalar(255), 2);
  putBlocksInEnvironment();

}


void RandomEnvironmentGenerator::initializeGrid(void)
{
  vector<vector<int>> circ_action_init{{0, 0}, {0, 0}, {0, 0}, {0, 0}};

  //Resizing environment grid
  environment_grid.resize(environment_width);
  for (int it = 0; it < environment_width; it++) {
    environment_grid[it].resize(environment_height);
  }

  //TODO: get this like trajectory_loop_function does
  for (int itx = 0; itx < environment_width; itx++) {
    for (int ity = 0; ity < environment_height; ity++) {
      environment_grid.at(itx).at(ity).is_corridor_present = false;
      environment_grid.at(itx).at(ity).is_agent_present = false;
      environment_grid.at(itx).at(ity).circ_action = circ_action_init;
    }
  }
}

void RandomEnvironmentGenerator::initializeAgents(void)
{

  current_agent_positions.resize(2);
  // initial robot positions, place agent where they are
  vector<vector<int>> circ_action_init{{0, 1}, {1, 0}, {0, -1}, { -1, 0}};

  initial_bot_positions.resize(2);
  initial_bot_positions.at(0).resize(2);
  initial_bot_positions.at(1).resize(2);

  vector<int> bot_1_position = {3, 0};
  vector<int> bot_2_position = {6, 9};

  initial_bot_positions.at(0) = bot_1_position;
  initial_bot_positions.at(1) = bot_2_position;


  for (int it = 0; it < 2; it++) {
    environment_grid.at(initial_bot_positions.at(it).at(0)).at(initial_bot_positions.at(it).at(1)).is_agent_present = true;

    std::rotate(circ_action_init.begin(), circ_action_init.begin() + it, circ_action_init.end());
    environment_grid.at(initial_bot_positions.at(it).at(0)).at(initial_bot_positions.at(it).at(1)).circ_action = circ_action_init;

  }
}

void RandomEnvironmentGenerator::findAgents(void)
{
  current_agent_positions.clear();

  int k = 0;
  for (int itx = 0; itx < environment_width; itx++) {
    for (int ity = 0; ity < environment_height; ity++) {
      if (environment_grid.at(itx).at(ity).is_agent_present) {
        current_agent_positions.resize(k + 1);
        current_agent_positions.at(k).resize(2);
        current_agent_positions.at(k).at(0) = itx;
        current_agent_positions.at(k).at(1) = ity;
        k++;
      }
    }
  }

}

void RandomEnvironmentGenerator::decideNextAction(std::vector<int> current_bot_position)
{
  float random_percentage = static_cast <float>(rand()) / static_cast <float>(RAND_MAX);
  float percentage_rest = 1.0f - change_agent_gostraight;

  vector<vector<int>>circ_action_temp = environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action;

  string state;
  if (random_percentage <= change_agent_gostraight) {
    state = "GO_STRAIGHT";
  } else if (random_percentage > change_agent_gostraight &&
             random_percentage <= change_agent_gostraight + percentage_rest / 2.0f) {
    state = "GO_LEFT";
    std::rotate(circ_action_temp.begin(), circ_action_temp.begin() + 1, circ_action_temp.end());

  } else if (random_percentage > change_agent_gostraight + percentage_rest / 2.0f &&
             random_percentage <= 1) {
    state = "GO_RIGHT";
    std::rotate(circ_action_temp.rbegin(), circ_action_temp.rbegin() + 1, circ_action_temp.rend());
  }

  environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action = circ_action_temp;


}

int mod(int a, int b)
{ return (a % b + b) % b; }

void RandomEnvironmentGenerator::setNextLocation(std::vector<int> current_bot_position)
{
  vector<vector<int>>circ_action_temp = environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action;
  vector<int> next_location{current_bot_position.at(0) + circ_action_temp.at(0).at(0), current_bot_position.at(1) + circ_action_temp.at(0).at(1)};


  vector<int> next_location_corrected{mod(next_location.at(0), environment_width), mod(next_location.at(1), environment_height)};

  environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).is_agent_present = false;
  environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).is_corridor_present = true;
  environment_grid.at(next_location_corrected.at(0)).at(next_location_corrected.at(1)).is_agent_present = true;
  environment_grid.at(next_location_corrected.at(0)).at(next_location_corrected.at(1)).circ_action = circ_action_temp;



}
float RandomEnvironmentGenerator::getCorridorPercentage()
{
  int count_corridor = 0;
  for (int itx = 0; itx < environment_width; itx++) {
    for (int ity = 0; ity < environment_height; ity++) {
      if (environment_grid.at(itx).at(ity).is_corridor_present) {
        count_corridor++;
      }
    }
  }

  return (float)count_corridor / (float)(environment_width * environment_height);
}
void RandomEnvironmentGenerator::makeBinaryImageCorridors()
{

  for (int itx = 0; itx < environment_width; itx++) {
    for (int ity = 0; ity < environment_height; ity++) {
      // cout << environment_grid.at(itx).at(ity).is_corridor_present << " ";
      if (environment_grid.at(itx).at(ity).is_corridor_present) {
        bin_corridor_img.at<uchar>(itx, ity) = 255;
      }
    }
  }
}


void RandomEnvironmentGenerator::checkConnectivity()
{
  Mat labels;
  connectedComponents(bin_corridor_img, labels, 4, CV_16U);
  ushort label_at_first_location = labels.at<ushort>(initial_bot_positions.at(0).at(0), initial_bot_positions.at(0).at(1));

  for (int it = 1; it < initial_bot_positions.size(); it++) {
    ushort label_at_second_location = labels.at<ushort>(initial_bot_positions.at(it).at(0), initial_bot_positions.at(it).at(1));
    if (label_at_first_location == label_at_second_location) {
      corridors_are_connected = true;
    } else {
      corridors_are_connected = false;
      break;
    }
  }

}

void RandomEnvironmentGenerator::makeBoundariesCorridors()
{
  bin_corridor_img_large = Mat::zeros(environment_width * 20, environment_height * 20, CV_8UC1);
  resize(bin_corridor_img, bin_corridor_img_large, bin_corridor_img_large.size(), 0, 0, INTER_NEAREST);

  vector<vector<Point>> contours_coordinates;
  Mat hierarchy;

  findContours(bin_corridor_img_large, contours_coordinates, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  Scalar color = Scalar(255, 255, 255);

  corridor_contours_img = Mat::zeros(bin_corridor_img_large.size(), CV_8UC1);
  for (int i = 0; i < contours_coordinates.size(); i++) {
    drawContours(corridor_contours_img, contours_coordinates, i, color, 2, 8, hierarchy, 0, Point());
  }




}
void RandomEnvironmentGenerator::makeRooms()
{

  for (int itx = 0; itx < environment_width * 20; itx++) {
    for (int ity = 0; ity < environment_height * 20; ity++) {

      vector<int> coord_mod_rooms{itx % (int)(environment_width * 20 * room_percentage), ity % (int)(environment_height * 20 * room_percentage)};

      if ((coord_mod_rooms.at(0) == 0 || coord_mod_rooms.at(1) == 0))
        if (bin_corridor_img_large.at<uchar>(ity, itx) == 0) {
          rectangle(corridor_contours_img, Point(itx - 1, ity - 1), Point(itx + 1, ity + 1), Scalar(255), 1, 8, 0);
        }
    }
  }
}

void RandomEnvironmentGenerator::makeRandomOpenings()
{
  RNG rng(0xFFFFFFFF);
  int half_size_openings = 8;
  for (int it = 0; it < 30; it++) {
    vector<int> random_coordinate{rng.uniform(half_size_openings, environment_height * 20 - half_size_openings), rng.uniform(half_size_openings, environment_width * 20 - half_size_openings)};
    rectangle(corridor_contours_img, Point(random_coordinate.at(0) - half_size_openings, random_coordinate.at(1) - half_size_openings), Point(random_coordinate.at(0) + half_size_openings, random_coordinate.at(1) + half_size_openings), Scalar(0), CV_FILLED, 8, 0);

  }


}

void RandomEnvironmentGenerator::putBlocksInEnvironment()
{

  CBoxEntity *boxEntity;
  CVector3 boxEntitySize{0.1, 0.1, 0.5};
  CQuaternion boxEntityRot{0, 0, 0, 0};

  std::ostringstream box_name;


  int i = 0;
  for (int itx = 0; itx < environment_width * 20; itx++) {
    for (int ity = 0; ity < environment_height * 20; ity++) {
      if (corridor_contours_img.at<uchar>(ity, itx) == 255) {
        box_name.str("");
        box_name << "box" << (i);
        vector<double> argos_coordinates{(double)(itx - environment_width * 10) / 10.0f, (double)(ity - environment_height * 10) / 10.0f};
        CVector3 boxEntityPos{argos_coordinates.at(0), argos_coordinates.at(1), 0};
        boxEntity = new CBoxEntity(box_name.str(), boxEntityPos, boxEntityRot, false, boxEntitySize);
        AddEntity(*boxEntity);

        i++;
      }

    }
  }

}

REGISTER_LOOP_FUNCTIONS(RandomEnvironmentGenerator, "random_environment_generator");
