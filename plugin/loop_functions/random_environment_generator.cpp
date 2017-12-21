/*
 * random_environment_generator.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#include "random_environment_generator.h"
#include <chrono>


//#define ACCEPT_ENVIRONMENT

using namespace std;
using namespace cv;


RandomEnvironmentGenerator::RandomEnvironmentGenerator() :
  environment_width(10),
  environment_height(10),
  change_agent_gostraight(0.75f),
  wanted_corridor_percentage(0.4f),
  room_percentage(0.3f),
  total_boxes_generated(0),
  amount_of_openings(8),
  environment_accepted(false){}


void RandomEnvironmentGenerator::getRobotPositions()
{
  CSpace::TMapPerType& tFBMap = CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");

  for(CSpace::TMapPerType::iterator it = tFBMap.begin();
      it != tFBMap.end();
      ++it) {

     CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
     CVector3 pos_bot;
     pos_bot = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
     vector<int> initial_bot_position{0,0};
     initial_bot_position.at(0)=pos_bot.GetX()/2+environment_width/2;
     initial_bot_position.at(1)=pos_bot.GetY()/2+environment_height/2;
     initial_bot_positions.push_back(initial_bot_position);
  }
}

void RandomEnvironmentGenerator::Init(TConfigurationNode &t_node)
{
  //TODO use the params of loop functions, if they exist

  const CVector3& cArenaSize = CSimulator::GetInstance().GetSpace().GetArenaSize();

  environment_accepted =false;
  environment_width = (int)(cArenaSize.GetX()/2);
  environment_height=(int)(cArenaSize.GetY()/2);

  /* Go through them */
  int i = 0;

  getRobotPositions();
  initializeGrid();
  initializeAgents();
  bin_corridor_img = Mat::zeros(environment_width, environment_height, CV_8UC1);
  corridor_contours_img = Mat::zeros(bin_corridor_img_large.size(), CV_8UC1);

}


void RandomEnvironmentGenerator::ClearEnvironment()
{
  std::cout<<"clear environment"<<std::endl;
  CLoopFunctions loopfunction;

  if(total_boxes_generated!=0)
  {
    for(int i = 0;i<total_boxes_generated+1;i++){
      //auto start_time = std::chrono::high_resolution_clock::now();

      loopfunction.RemoveEntity(*boxEntities.at(i));
      /* auto end_time = std::chrono::high_resolution_clock::now();
     auto time = end_time - start_time;

     std::cout << "It took " <<
       std::chrono::duration_cast<std::chrono::microseconds>(time).count() << " to run.\n";*/
    }
  }
     boxEntities.clear();
     total_boxes_generated =0;
     environment_accepted =false;

}

void RandomEnvironmentGenerator::Reset(std::string file_name)
{

 cout<<"Regenerate Environment"<<endl;


    if(file_name.length()==0)
    {
      initial_bot_positions.clear();
      generateEnvironment();
    std::cout<<"random generated: "<<file_name<<std::endl;
    }
    else{
      generateEnvironmentFromFile(file_name);
     std::cout<<"generate from file_name: "<<file_name<<std::endl;
    }

}
void RandomEnvironmentGenerator::Destroy()
{
}

void RandomEnvironmentGenerator::generateEnvironment(void)
{
  corridors_are_connected = false;
  rng = cv::getTickCount();

  while(!environment_accepted){
    while (!corridors_are_connected) {
      getRobotPositions();
      initializeGrid();
      initializeAgents();
      bin_corridor_img = Mat::zeros(environment_width, environment_height, CV_8UC1);

      for (int it_total = 0; it_total < 100; it_total++) {

        findAgents();

        for (int it = 0; it < current_agent_positions.size(); it++) {
          decideNextAction(current_agent_positions.at(it));
          setNextLocation(current_agent_positions.at(it));

        }

/*        for (int itx = 0; itx < environment_width; itx++) {
           for (int ity = 0; ity < environment_height; ity++) {
               cout<<environment_grid.at(itx).at(ity).is_corridor_present<<" ";
           }
           cout<<" "<<endl;
         }*/

        if (getCorridorPercentage() > wanted_corridor_percentage) {
          break;
        }


      }

      checkConnectivity();

      if(!corridors_are_connected)
        cout<<"corridors are not connected!!"<<endl;
        rng = cv::getTickCount();
    }


    makeBinaryImageCorridors();
    makeBoundariesCorridors();
    makeRooms();
    makeRandomOpenings();



    cv::Rect border(cv::Point(0, 0), corridor_contours_img.size());

    rectangle(corridor_contours_img, border, Scalar(255), 3);
    cv::imwrite("environment.png",corridor_contours_img);



#ifdef ACCEPT_ENVIRONMENT

    namedWindow( "Environment", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Environment", corridor_contours_img );                   // Show our image inside it.
    char key = (char)waitKey(0);
    if(key=='y')
    {
      environment_accepted=true;
      break;
    }else
    {
      rng = cv::getTickCount();
      corridors_are_connected = false;
    }
#else
    environment_accepted=true;
#endif

  }

  putBlocksInEnvironment();

}

void RandomEnvironmentGenerator::generateEnvironmentFromFile(std::string file_name)
{

  bin_corridor_img_large = Mat::zeros(environment_width * 20, environment_height * 20, CV_8UC1);
  resize(bin_corridor_img, bin_corridor_img_large, bin_corridor_img_large.size(), 0, 0, INTER_NEAREST);
  corridor_contours_img = Mat::zeros(bin_corridor_img_large.size(), CV_8UC1);
  std::cout<<"size "<<bin_corridor_img_large.size()<<std::endl;

  cv::Mat read_img = cv::imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
  resize(read_img, corridor_contours_img, corridor_contours_img.size(), 0, 0, INTER_NEAREST);


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


  for (int it = 0; it < initial_bot_positions.size(); it++) {
    environment_grid.at(initial_bot_positions.at(it).at(1)).at(initial_bot_positions.at(it).at(0)).is_agent_present = true;

    std::rotate(circ_action_init.begin(), circ_action_init.begin() + std::rand()%4, circ_action_init.end());
    environment_grid.at(initial_bot_positions.at(it).at(1)).at(initial_bot_positions.at(it).at(0)).circ_action = circ_action_init;

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
  float random_percentage = rng.uniform(0.0f,1.0f);
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


void RandomEnvironmentGenerator::checkConnectivityOpenCV()
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


const int dx[] = {+1, 0, -1, 0};
const int dy[] = {0, +1, 0, -1};
void RandomEnvironmentGenerator::dfs(int x, int y, int current_label) {
  if (x < 0 || x == environment_width) return; // out of bounds
  if (y < 0 || y == environment_height) return; // out of bounds
  if (connectivity_labels.at(x).at(y) || !environment_grid.at(x).at(y).is_corridor_present) return; // already labeled or not marked with 1 in m

  // mark the current cell
  connectivity_labels.at(x).at(y) = current_label;

  // recursively mark the neighbors
  for (int direction = 0; direction < 4; ++direction)
    dfs(x + dx[direction], y + dy[direction], current_label);
}

void RandomEnvironmentGenerator::checkConnectivity()
{

  vector<int> linked;
  //Resizing connectivity_labels grid
  connectivity_labels.resize(environment_width);
  for (int it = 0; it < environment_width; it++) {
    connectivity_labels[it].resize(environment_height);
  }

  for (int itx = 0; itx < environment_width; itx++) {
    for (int ity = 0; ity < environment_height; ity++) {
      connectivity_labels.at(itx).at(ity)=0;
    }
  }
  vector<int> neighbors_labels;
  int next_label = 0;

  int component = 0;
  for (int itx = 0; itx < environment_width; itx++) {
    for (int ity = 0; ity < environment_height; ity++) {
      if (!connectivity_labels.at(itx).at(ity)&&environment_grid.at(itx).at(ity).is_corridor_present)
        {
        dfs(itx, ity, ++component);
        }
    }
  }

/*
  for (int itx = 0; itx < environment_width; itx++) {
     for (int ity = 0; ity < environment_height; ity++) {
         cout<<connectivity_labels.at(itx).at(ity)<<" ";
     }
     cout<<" "<<endl;
   }
*/

  int label_at_first_location = connectivity_labels.at(initial_bot_positions.at(0).at(1)).at(initial_bot_positions.at(0).at(0));

  for (int it = 1; it < initial_bot_positions.size(); it++) {
    int label_at_second_location = connectivity_labels.at(initial_bot_positions.at(it).at(1)).at(initial_bot_positions.at(it).at(0));
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

  dilate(bin_corridor_img_large, bin_corridor_img_large, Mat(), Point(-1, -1), 2, 1, 1);
  dilate(bin_corridor_img_large, bin_corridor_img_large, Mat(), Point(-1, -1), 2, 1, 1);

  for(int it =0; it<initial_bot_positions.size();it++)
    rectangle(bin_corridor_img_large, Point(initial_bot_positions.at(it).at(0)*20 + 10 - 20, initial_bot_positions.at(it).at(1)*20 + 10 - 20),
        Point(initial_bot_positions.at(it).at(0)*20 + 10 + 20, initial_bot_positions.at(it).at(1)*20 + 10 + 20), Scalar(255), CV_FILLED, 8, 0);
/*  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", bin_corridor_img_large );                   // Show our image inside it.

  waitKey(0);*/

  vector<vector<Point>> contours_coordinates;
  Mat hierarchy;

  findContours(bin_corridor_img_large, contours_coordinates, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  Scalar color = Scalar(255, 255, 255);

  corridor_contours_img = Mat::zeros(bin_corridor_img_large.size(), CV_8UC1);
  for (int i = 0; i < contours_coordinates.size(); i++) {

    drawContours(corridor_contours_img, contours_coordinates, i, color, 1  , LINE_4, hierarchy, 0);
  }

    dilate(corridor_contours_img, corridor_contours_img, Mat(), Point(-1, -1), 2, 1, 1);

//Mat element = getStructuringElement(MORPH_RECT, Size(2, 2), Point(1,1) );



}
void RandomEnvironmentGenerator::makeRooms()
{
  Mat bin_corridor_img_large_inv = Mat::zeros(environment_width * 20, environment_height * 20, CV_8UC1);
  bitwise_not(bin_corridor_img_large,bin_corridor_img_large_inv);
  corridor_contours_img.copyTo(corridor_contours_img_save);
  vector<vector<Point>> contours_coordinates;
  Mat hierarchy;

  findContours(bin_corridor_img_large_inv, contours_coordinates, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  for(int i = 0; i <contours_coordinates.size();i++ )
  {
    Rect boundRect=boundingRect(contours_coordinates.at(i));
       for(int itx = boundRect.x; itx<(boundRect.x + boundRect.width); itx++)
    {
      for(int ity = boundRect.y; ity<(boundRect.y + boundRect.height); ity++)
      {

        vector<int> coord_mod_rooms{(itx-boundRect.x) % (int)(boundRect.width /2), (ity-boundRect.y) % (int)(boundRect.height /2)};
        if( boundRect.width<(float)environment_width * 20*room_percentage)
          coord_mod_rooms.at(0) = 1;
        if(boundRect.height<(float)environment_height * 20*room_percentage)
          coord_mod_rooms.at(1) = 1;

    if ((coord_mod_rooms.at(0) == 0 || coord_mod_rooms.at(1) == 0))
        if (bin_corridor_img_large.at<uchar>(ity, itx) == 0) {
          rectangle(corridor_contours_img, Point(itx - 1, ity - 1), Point(itx + 1, ity + 1), Scalar(255), 1, 8, 0);
        }
      }
    }
  }


/*
  for (int itx = 0; itx < environment_width * 20; itx++) {
    for (int ity = 0; ity < environment_height * 20; ity++) {

      vector<int> coord_mod_rooms{itx % (int)(environment_width * 20 * room_percentage), ity % (int)(environment_height * 20 * room_percentage)};

      if ((coord_mod_rooms.at(0) == 0 || coord_mod_rooms.at(1) == 0))
        if (bin_corridor_img_large.at<uchar>(ity, itx) == 0) {
          rectangle(corridor_contours_img, Point(itx - 1, ity - 1), Point(itx + 1, ity + 1), Scalar(255), 1, 8, 0);
        }
    }
  }*/
}

void RandomEnvironmentGenerator::makeRandomOpenings()
{
  RNG rng(cv::getTickCount());
  int half_size_openings = 13;
  int erosion_size = 1;
  Mat element = getStructuringElement(cv::MORPH_CROSS,
         cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
         cv::Point(erosion_size, erosion_size) );
  Mat corridor_contours_img_temp = Mat::zeros(corridor_contours_img.size(), CV_8UC1);



  erode(corridor_contours_img_save, corridor_contours_img_temp, element, Point(-1, -1), 2, 1, 1);


  for (int itx = 0; itx < environment_width * 20; itx++) {
    for (int ity = 0; ity < environment_height * 20; ity++) {
        if(corridor_contours_img_temp.at<uchar>(ity, itx)==255&&  rng.uniform(1, 1000)<amount_of_openings
            &&itx!=0&&itx!=environment_width* 20
            &&itx!=0&&itx!=environment_height* 20)
        {
          corridor_contours_img.at<uchar>(ity, itx)==0;
          rectangle(corridor_contours_img, Point(itx - half_size_openings, ity - half_size_openings), Point(itx + half_size_openings, ity + half_size_openings), Scalar(0), CV_FILLED, 8, 0);

        }
    }
  }

/*
  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", corridor_contours_img );                   // Show our image inside it.

  waitKey(0);*/
/*  for (int it = 0; it < amount_of_openings; it++) {
    vector<int> random_coordinate{rng.uniform(half_size_openings, environment_height * 20 - half_size_openings), rng.uniform(half_size_openings, environment_width * 20 - half_size_openings)};
    rectangle(corridor_contours_img, Point(random_coordinate.at(0) - half_size_openings, random_coordinate.at(1) - half_size_openings), Point(random_coordinate.at(0) + half_size_openings, random_coordinate.at(1) + half_size_openings), Scalar(0), CV_FILLED, 8, 0);
    //circle(corridor_contours_img, Point(random_coordinate.at(0), random_coordinate.at(1)), half_size_openings, Scalar(0), -1, 8, 0);


  }*/


}

void RandomEnvironmentGenerator::putBlocksInEnvironment()
{

  CBoxEntity* boxEntity;
  CVector3 boxEntitySize{0.1, 0.1, 0.5};
  CQuaternion boxEntityRot{0, 0, 0, 0};

  std::ostringstream box_name;

/*  if(i<total_boxes_generated)
  {
    loop_function.MoveEntity(boxEntities.at(i)->GetEmbodiedEntity(),boxEntityPos,boxEntityRot);
  }else
  {
    CBoxEntity *boxEntity = new CBoxEntity(box_name.str(), boxEntityPos, boxEntityRot, false, boxEntitySize);
    loop_function.AddEntity(*boxEntity);
    boxEntities.push_back(boxEntity);
  }*/

  CLoopFunctions loopfunction;
  int i = 0;
  for (int itx = 0; itx < environment_width * 20; itx++) {
    for (int ity = 0; ity < environment_height * 20; ity++) {
      if (corridor_contours_img.at<uchar>(ity, itx) == 255) {
        box_name.str("");
        box_name << "box" << (i);
        vector<double> argos_coordinates{(double)(itx - environment_width * 10) / 10.0f, (double)(ity - environment_height * 10) / 10.0f};
        CVector3 boxEntityPos{argos_coordinates.at(0), argos_coordinates.at(1), 0};
        boxEntity = new CBoxEntity(box_name.str(), boxEntityPos, boxEntityRot, false, boxEntitySize);

        loopfunction.AddEntity(*boxEntity);

        boxEntities.push_back(boxEntity);


        i++;
      }

    }
  }
  total_boxes_generated=i-1;

}
