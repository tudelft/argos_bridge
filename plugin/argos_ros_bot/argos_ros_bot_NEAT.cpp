// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot_NEAT.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <sstream>
#include <string>
#include <iostream>

#include <ros/callback_queue.h>

#include <math.h>
#include "tf/LinearMath/Transform.h"


using namespace std;

/****************************************/
/****************************************/

CArgosRosBotNEAT::CArgosRosBotNEAT() :
      m_pcWheels(NULL),
      m_pcProximity(NULL),
      //m_pcOmniCam(NULL),
      m_pcRangeBearing(NULL),
      //  m_pcGripper(NULL),
      stopWithoutSubscriberCount(10),
      stepsSinceCallback(0),
      leftSpeed(0),
      rightSpeed(0)//,
//  gripping(false)
{

   std::ifstream iFile ("ibug_working_directory/temp/temp_gnome");

   char curword[20];
   int id;

   iFile >> curword;
   iFile >> id;

   NEAT::Genome *start_genome = new NEAT::Genome(id,iFile);
   iFile.close();

   NEAT::Organism *neatOrg = new NEAT::Organism(0.0,start_genome,1);
   m_net = neatOrg->net;

   //Need to set size here otherwise seg fault further on...
   net_inputs.resize(m_net->inputs.size());
   net_outputs.resize(m_net->outputs.size());

}

void CArgosRosBotNEAT::Init(TConfigurationNode& t_node) {

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  //m_pcOmniCam = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  //m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
  //m_pcOmniCam->Enable();

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}

void CArgosRosBotNEAT::ControlStep() {

  if(GetId()=="bot0")
    {
      /* Get readings from proximity sensor */
      const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

      // Get readings from range and bearing sensor
      const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRangeBearing->GetReadings();

      //int h =0;
      std::cout << "Range & Bearing size: " << tRabReads.size() << std::endl;
      for(int i = 0; i < tRabReads.size(); i++) {
         //std::cout << tRabReads[i].Range << std::endl;
        net_inputs[i+1] = tRabReads[i].Range;
        //h++;
        //std::cout << "Read" << i << std::endl;
      }
      //std::cout << tProxReads.size() << std::endl;
      for(int i = 0; i < tProxReads.size(); i++) {
        //std::cout << tProxReads[i].Value << std::endl;
        net_inputs[i+tRabReads.size()+1] = tProxReads[i].Value;
        //h++;
        //std::cout << h << std::endl;
        //std::cout << "Read" << i <<std::endl;
      }
      //std::cout << std::endl;

      // for(int i =0; i < net_inputs.size(); i++) {
      //     std::cout << net_inputs[i] << std::endl;
      // }

      m_net->load_sensors(net_inputs);

      if (!(m_net->activate())) std::cout << "Inputs disconnected from output!";

      //Get outputs
      std::vector<NEAT::NNode*>::iterator it;

      for(it = m_net->outputs.begin(); it != m_net->outputs.end(); it++) {

         net_outputs[it-m_net->outputs.begin()] = (*it)->activation;

      }

      //std::cout << net_outputs[0] << std::endl;
      //std::cout << net_outputs[1] << std::endl;

      ConvertDifferentialDriveToSpeed(net_outputs[0], net_outputs[1]);

      // Wait for any callbacks to be called.
      m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);

    }

}




void CArgosRosBotNEAT::ConvertDifferentialDriveToSpeed(Real linear_x, Real angular_z) {

  Real v = linear_x;// Forward speed
  Real w = angular_z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}

void CArgosRosBotNEAT::Reset() {

   //Read in new genome

   std::ifstream iFile ("ibug_working_directory/temp/temp_gnome");
   //std::ifstream iFile ("ibug_working_directory/temp/footbot_start_24");    //Nothing to do with file

   char curword[20];
   int id;

   iFile >> curword;
   iFile >> id;

   NEAT::Genome *start_genome = new NEAT::Genome(id,iFile);
   iFile.close();

   NEAT::Organism *neatOrg = new NEAT::Organism(0.0,start_genome,1);
   m_net = neatOrg->net;

   net_inputs[0] = 1.0;                            //Bias node

   //Initialise inputs
   for(int i = 1; i < m_net->inputs.size(); i++) {

       net_inputs[i] = 1.0;
       net_outputs[i] = 1.0;

   }

}



/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CArgosRosBotNEAT, "argos_ros_bot_neat_controller")
