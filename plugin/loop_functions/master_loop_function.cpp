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
extern std::string file_name_env;
extern int file_name_env_number;

#define RANDOM_ENVIRONMENT_GEN_ON true

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


MasterLoopFunction::MasterLoopFunction() : prev_file_name_env_number(-1) {}
MasterLoopFunction::~MasterLoopFunction() {}

/*Init: Get all the footbot entities and initialize distance for fitness score
 *
 */
void MasterLoopFunction::Init(TConfigurationNode& t_node)
{

  fitnessScoreLoopFunction.Init(t_node);
  trajectoryLoopFunction.Init(t_node);
#if(RANDOM_ENVIRONMENT_GEN_ON)
 randomEnvironmentGenerator.Init( t_node);
#endif
}


/*Reset: reinitialize fitnesscore
 *
 */
void MasterLoopFunction::Reset(){

   //if (prev_file_name_env_number == -1) prev_file_name_env_number = file_name_env_number;

   //std::cout << "Reset.." << std::endl;

   SetRandomRobotOrientation();

  fitnessScoreLoopFunction.Reset();
  trajectoryLoopFunction.Reset();
#if(RANDOM_ENVIRONMENT_GEN_ON)
  if(regen_env==1) {
    std::string file_name_empty = "";
    randomEnvironmentGenerator.Reset(file_name_empty);
  }else if(regen_env==3) {
   //std::cout << "FILE NAME: " << file_name_env << std::endl;
    if(file_name_env_number != prev_file_name_env_number) randomEnvironmentGenerator.Reset(file_name_env);
    SetRobotPosBasedOnMap(file_name_env_number);
}
#endif

   //std::cout << "..resetted" << std::endl;

   prev_file_name_env_number = file_name_env_number;
}

void MasterLoopFunction::SetRobotPosBasedOnMap(int map_type) {

   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {

      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      CEmbodiedEntity*  embEntity = GetEmbodiedEntity(pcFB);

      CRadians cOrient = (CRadians)(((double)rand() / RAND_MAX) * 2 * M_PI);
      double xPos, yPos;

      if(pcFB->GetId()=="bot0") {

         switch(map_type) {

            //Simple env maps

            // case 1:
            //
            //    xPos = -4.0;
            //    yPos = -4.0;
            //    cOrient = (CRadians)5*M_PI/4;
            //    break;
            //
            // case 2:
            //
            //    xPos = -4.0;
            //    yPos = 4.0;
            //    cOrient = (CRadians)3*M_PI/4;
            //    break;

            //Experiment maps

            case 1:

               xPos = -4.0;
               yPos = -4.0;
               break;

            case 2:

               xPos = -4.0;
               yPos = -4.0;
               break;

            case 3:

               xPos = -4.0;
               yPos = -4.0;
               break;

         }

      } else if(pcFB->GetId()=="bot1") {

         switch(map_type) {

            //Simple env maps

            // case 1:
            //
            //    xPos = 4.0;
            //    yPos = 4.0;
            //    break;
            //
            // case 2:
            //
            //    xPos = 4.0;
            //    yPos = -4.0;
            //    break;

            case 1:

               xPos = 4.0;
               yPos = 4.0;
               break;

            case 2:

               xPos = 4.0;
               yPos = 4.0;
               break;

            case 3:

               xPos = 4.0;
               yPos = 4.0;
               break;

         }

      }

      SInitSetup robot_allocation;
      robot_allocation.Orientation.FromEulerAngles(
         cOrient,        // rotation around Z
         CRadians::ZERO, // rotation around Y
         CRadians::ZERO  // rotation around X
         );
      robot_allocation.Position.Set(xPos, yPos, 0.0);

      while (!MoveEntity(
             *embEntity,     // move the body of the robot
             robot_allocation.Position,                // to this position
             robot_allocation.Orientation,             // with this orientation
             false                                 // this is not a check, leave the robot there
         )) {

             LOGERR << "Can't move robot in <"
                       << robot_allocation.Position
                       << ">, <"
                       << robot_allocation.Orientation
                       << ">"
                       << std::endl;

      }

   }

}

void MasterLoopFunction::SetRandomRobotOrientation() {

   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {

      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      CEmbodiedEntity*  embEntity = GetEmbodiedEntity(pcFB);

      if(pcFB->GetId()=="bot0") {

         SInitSetup robot_allocation;

         CRadians cOrient = (CRadians)(((double)rand() / RAND_MAX) * 2 * M_PI);

         //Get position from XML file
         CVector3 rob_pos = GetRobotPositionFromXML();
         robot_allocation.Position.Set(rob_pos.GetX(), rob_pos.GetY(), rob_pos.GetZ());
         robot_allocation.Orientation.FromEulerAngles(
            cOrient,        // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
            );

         while (!MoveEntity(
                *embEntity,     // move the body of the robot
                robot_allocation.Position,                // to this position
                robot_allocation.Orientation,             // with this orientation
                false                                 // this is not a check, leave the robot there
            )) {

                LOGERR << "Can't move robot in <"
                          << robot_allocation.Position
                          << ">, <"
                          << robot_allocation.Orientation
                          << ">"
                          << std::endl;

             }

       }

   }

}


/*PreStep: Execute a function at every step of the simulation
 * *
 */
void MasterLoopFunction::PreStep()
{
  trajectoryLoopFunction.PostStep();
  fitnessScoreLoopFunction.PreStep();

  // fitnessScoreLoopFunction.PreStep();
  // /* Get the map of all foot-bots from the space */
  // CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
  // /* Go through them */
  // for(CSpace::TMapPerType::iterator it = tFBMap.begin();
  //     it != tFBMap.end();
  //     ++it) {
  //
  //    /* Create a pointer to the current foot-bot */
  //    CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
  //    CEmbodiedEntity*  embEntity = GetEmbodiedEntity(pcFB);
  //
  //    if(pcFB->GetId()=="bot0") {
  //
  //       std::cout << embEntity->GetOriginAnchor().Position << std::endl;
  //
  //     }
  //
  // }
}

/*PreStep: Before the simulation,
 *
 */
void MasterLoopFunction::PostExperiment()
{
  fitnessScoreLoopFunction.PostExperiment();
  trajectoryLoopFunction.PostExperiment();

}

CEmbodiedEntity* MasterLoopFunction::GetEmbodiedEntity(CEntity* pc_entity) {
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


CVector3 MasterLoopFunction::GetRobotPositionFromXML() {

   // Get robot position
   try {

      argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();

      argos::TConfigurationNode& arena_node = GetNode(cSimulator.GetConfigurationRoot(), "arena");

      argos::TConfigurationNodeIterator itArenaItem;
      for(itArenaItem = itArenaItem.begin(&arena_node);
          itArenaItem != itArenaItem.end();
          ++itArenaItem) {

             std::string arena_item_name;
             GetNodeAttribute(*itArenaItem, "id", arena_item_name);

             if(arena_item_name.compare("bot0") == 0) {

                argos::TConfigurationNode& body_node = GetNode(*itArenaItem, "body");

                CVector3 position;

                GetNodeAttributeOrDefault(body_node, "position", position, CVector3());

                return position;

            }

      }

   }

   catch(CARGoSException& ex) {

      std::cout << "Can't get position of robot from XML file" << std::endl;

   }


}


REGISTER_LOOP_FUNCTIONS(MasterLoopFunction, "master_loop_function");
