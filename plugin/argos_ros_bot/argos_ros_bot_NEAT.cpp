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

#define GRADIENT_SENSOR_ON true
#define PROX_SENSOR_ON true

/****************************************/
/****************************************/

CArgosRosBotNEAT::CArgosRosBotNEAT() :
      m_pcWheels(NULL),
      m_pcProximity(NULL),
      m_pcRangeBearing(NULL),
      stopWithoutSubscriberCount(10),
      stepsSinceCallback(0),
      leftSpeed(0),
      rightSpeed(0),
      NET_INPUT_LOWER_BOUND(0.0),
      NET_INPUT_UPPER_BOUND(1.0),
      RANGE_SENSOR_LOWER_BOUND(0.0),
      RANGE_SENSOR_UPPER_BOUND(1350.0),
      NET_OUTPUT_LOWER_BOUND(0.0),
      NET_OUTPUT_UPPER_BOUND(1.0),
      MIN_LINEAR_VEL(0.0),
      MAX_LINEAR_VEL(0.075),
      MIN_ANGULAR_VEL(0.0),
      MAX_ANGULAR_VEL(40),
      PROX_SENSOR_LOWER_BOUND(0.0),
      PROX_SENSOR_UPPER_BOUND(0.1),
      BEARING_SENSOR_LOWER_BOUND(-M_PI),
      BEARING_SENSOR_UPPER_BOUND(M_PI),
      MAX_WHEEL_SPEED(10),
      MIN_WHEEL_SPEED(-10),
      memory(1)      //Memory vector size as input
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

   //if(GetId()=="bot0") memory.reset(new ENTMMemory(1));

}

void CArgosRosBotNEAT::Init(TConfigurationNode& t_node) {

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

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

      //Include inputs for both the range and the bearing
      for(int i = 0; i < tRabReads.size(); i++) {
         net_inputs[(i*2)+1] = mapValueIntoRange(tRabReads[i].Range,
                                            RANGE_SENSOR_LOWER_BOUND, RANGE_SENSOR_UPPER_BOUND,
                                            NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
         if(GRADIENT_SENSOR_ON) {
            //   net_inputs[(i*2)+2] = mapValueIntoRange(tRabReads[i].HorizontalBearing.GetValue(),
            //                                       BEARING_SENSOR_LOWER_BOUND, BEARING_SENSOR_UPPER_BOUND,
            //                                       NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
            net_inputs[(i*2)+2] = mapHorizontalAngle(tRabReads[i].HorizontalBearing.GetValue());
         }
      }

      //std::cout << net_inputs.size() << std::endl;

      //Proximity sensor inputs
      if(PROX_SENSOR_ON) {
         for(int i = 0; i < tProxReads.size(); i++) {
            net_inputs[i+(tRabReads.size()*2)+1] = mapValueIntoRange(tProxReads[i].Value,
                                                                     PROX_SENSOR_LOWER_BOUND, PROX_SENSOR_UPPER_BOUND,
                                                                     NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
         }
      }

      // std::cout << "----------" <<std::endl;
      // //Net input testing
      // for(int i =0; i < net_inputs.size(); i++) {
      //     std::cout << net_inputs[i] << std::endl;
      // }
      // std::cout << "----------" <<std::endl;

      m_net->load_sensors(net_inputs);

      if (!(m_net->activate())) std::cout << "Inputs disconnected from output!";

      //Get outputs

      //Linear velocity - mapped to a maximum speed

      // net_outputs[0] = mapValueIntoRange(m_net->outputs[0]->activation,
      //                                    NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
      //                                    MIN_LINEAR_VEL, MAX_LINEAR_VEL);
      //
      // //Angular velocity - mapped to a maximum turning speed
      // net_outputs[1] = mapValueIntoRange(m_net->outputs[1]->activation,
      //                                    NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
      //                                    MIN_ANGULAR_VEL, MAX_ANGULAR_VEL);

      leftSpeed = mapValueIntoRange(m_net->outputs[0]->activation,
                                          NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
                                          MIN_WHEEL_SPEED, MAX_WHEEL_SPEED);

      rightSpeed = mapValueIntoRange(m_net->outputs[1]->activation,
                                          NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
                                          MIN_WHEEL_SPEED, MAX_WHEEL_SPEED);

      //ConvertDifferentialDriveToSpeed(net_outputs[0], net_outputs[1]);
      //ConvertDifferentialDriveToSpeed(0.075, 50);

      //leftSpeed = 7.5;
      //rightSpeed = 7.5;
      //std::cout << "\n" << leftSpeed << " " << rightSpeed << std::endl;

      m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);

    }

}


double CArgosRosBotNEAT::mapHorizontalAngle(double angle) {

   return abs(1/M_PI * angle);

}

void CArgosRosBotNEAT::ConvertDifferentialDriveToSpeed(Real linear_x, Real angular_z) {

  Real v = linear_x * 100;// Forward speed
  Real w = angular_z; // Rotational speed

  //34.34 with lin_vel = 1.0 and angular_vel = 0.0

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = v - HALF_BASELINE * w;
  rightSpeed = v + HALF_BASELINE * w;

  stepsSinceCallback = 0;
}

double CArgosRosBotNEAT::mapValueIntoRange(const double input, const double input_start,
                                           const double input_end, const double output_start,
                                           const double output_end) {

   if(input > input_end) return 1.0;

   double slope = (output_end - output_start) / (input_end - input_start);
   double output = output_start + slope * (input - input_start);

   return output;

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
   // for(int i = 1; i < m_net->inputs.size(); i++) {
   //
   //     net_inputs[i] = 1.0;
   //     net_outputs[i] = 1.0;
   //
   // }

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
