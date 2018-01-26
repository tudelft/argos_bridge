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

#include <chrono>


using namespace std;

#define BEARING_SENSOR_ON true
#define PROX_SENSOR_ON true
#define LIN_VEL_ON false

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
      prev_ang_vel(0),
      range_tminus1(0),
      range_tminus2(0),
      first_time_step(true),
      NET_INPUT_LOWER_BOUND(0.0),
      NET_INPUT_UPPER_BOUND(1.0),
      RANGE_SENSOR_LOWER_BOUND(0.0),
      RANGE_SENSOR_UPPER_BOUND(1350.0),
      NET_OUTPUT_LOWER_BOUND(0.0),
      NET_OUTPUT_UPPER_BOUND(1.0),
      MIN_LINEAR_VEL(0.0),
      MAX_LINEAR_VEL(10),
      MIN_ANGULAR_VEL(-50),
      MAX_ANGULAR_VEL(50),
      PROX_SENSOR_LOWER_BOUND(0.0),
      PROX_SENSOR_UPPER_BOUND(0.1),
      BEARING_SENSOR_LOWER_BOUND(-M_PI),
      BEARING_SENSOR_UPPER_BOUND(M_PI),
      MAX_WHEEL_SPEED(10),
      MIN_WHEEL_SPEED(-10)
      //memory(1)      //Memory vector size as input
{

   std::ifstream iFile ("ibug_working_directory/temp/temp_gnome");

   char curword[20];
   int id;

   iFile >> curword;
   iFile >> id;

   NEAT::Genome *start_genome = new NEAT::Genome(id,iFile);
   //NEAT::Genome start_genome = NEAT::Genome(id,iFile);
   iFile.close();

   //NEAT::Organism *neatOrg = new NEAT::Organism(0.0,&start_genome,1);
   neatOrg = new NEAT::Organism(0.0,start_genome,1);
   m_net = neatOrg->net;

   //Need to set size here otherwise seg fault further on...
   // net_inputs.resize(m_net->inputs.size());
   // net_outputs.resize(m_net->outputs.size());

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

   //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

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

         if(BEARING_SENSOR_ON) {
            //Bearing sensor 1-0.5-1

            //   net_inputs[(i*2)+2] = mapValueIntoRange(tRabReads[i].HorizontalBearing.GetValue(),
            //                                       BEARING_SENSOR_LOWER_BOUND, BEARING_SENSOR_UPPER_BOUND,
            //                                       NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);

            //Bearing sensor 1-0-1

            //net_inputs[(i*2)+2] = mapHorizontalAngle(tRabReads[i].HorizontalBearing.GetValue());

            //Bearing sensor - left and right bearing sensors

            double bearing = tRabReads[i].HorizontalBearing.GetValue();
            double left_bearing_angle, right_bearing_angle;

            //Get the angular distances for each of the bearing sensors and normalise
            if(bearing < 0) {
               left_bearing_angle = mapValueIntoRange(abs(bearing),
                                                      0, 2*M_PI,
                                                      NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
               right_bearing_angle = 1 - left_bearing_angle;
            } else {
               right_bearing_angle = mapValueIntoRange(bearing,
                                                      0, 2*M_PI,
                                                      NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
               left_bearing_angle = 1 - right_bearing_angle;
            }

            //std::cout << right_bearing_angle << std::endl;
            net_inputs[(i*2)+2] = right_bearing_angle;
            net_inputs[(i*2)+3] = left_bearing_angle;

         } else {

            //Take previous angular velocity as input
            net_inputs[(i*2)+2] = prev_ang_vel;

            //If it is the first time step, set the previous ranges to the
            //same as the first range rather than 0, otherwise the derivative
            //makes it seem like the range has instantly changed.
            if(first_time_step) {

               range_tminus1 = net_inputs[1];
               range_tminus2 = net_inputs[1];

               first_time_step = false;

            }

            //Take first derivative of range as input
            net_inputs[(i*2)+3] = net_inputs[1] - range_tminus1;

            //Take second derivative of range as input
            net_inputs[(i*2)+4] = (net_inputs[1] - range_tminus1) - (range_tminus1 - range_tminus2);

            std::cout << "R: " << net_inputs[1] << std::endl;
            std::cout << "Rt-1: " << range_tminus1 << std::endl;
            std::cout << "Rt-2: " << range_tminus2 << std::endl;
            std::cout << "R1deriv: " << net_inputs[1] - range_tminus1 << std::endl;
            std::cout << "R2deriv: " << (net_inputs[1] - range_tminus1) - (range_tminus1 - range_tminus2) << std::endl;
            std::cout << "----------------" << std::endl;

         }
      }

      //std::cout << net_inputs.size() << std::endl;

      //Proximity sensor inputs
      if(PROX_SENSOR_ON) {

         int net_input_index;
         for(int i = 0; i < tProxReads.size(); i++) {

            if(BEARING_SENSOR_ON) net_input_index = i+(tRabReads.size()*3)+1;
            else net_input_index = i+(tRabReads.size()*4)+1;

            //Inverted laser
            double reading = tProxReads[i].Value;
            if(reading == 0) net_inputs[net_input_index] = 0;
            else net_inputs[net_input_index] = NET_INPUT_UPPER_BOUND - mapValueIntoRange(tProxReads[i].Value,
                                                                     PROX_SENSOR_LOWER_BOUND, PROX_SENSOR_UPPER_BOUND,
                                                                     NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);

         }
      }

      //  std::cout << "----------" <<std::endl;
      //Net input testing
      // for(int i = 0; i < net_inputs.size(); i++) {
      //     std::cout << net_inputs[i] << std::endl;
      // }
      // std::cout << "----------" <<std::endl;

      m_net->load_sensors(net_inputs);

      if (!(m_net->activate())) std::cout << "Inputs disconnected from output!";

      //Get outputs

      if(LIN_VEL_ON) {

         //Linear velocity - mapped to a maximum speed
         //std::cout << m_net->outputs[0]->activation << std::endl;
         double mapped_lin_vel = mapValueIntoRange(m_net->outputs[0]->activation,
                                            NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
                                            MIN_LINEAR_VEL, MAX_LINEAR_VEL);
         //std::cout << mapped_lin_vel << std::endl;
         //mapped_lin_vel = 10;
         //Angular velocity - mapped to a maximum turning speed
         //std::cout << m_net->outputs[1]->activation << std::endl;
         double mapped_ang_vel = mapValueIntoRange(m_net->outputs[1]->activation,
                                            NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
                                            MIN_ANGULAR_VEL, MAX_ANGULAR_VEL);
         //std::cout << m_net->outputs[1]->activation << std::endl;
         //mapped_ang_vel = 100;
         //This function changes leftSpeed and rightSpeed
         ConvertLinVelToWheelSpeed(mapped_lin_vel, mapped_ang_vel);

         //Set currently angular vel as previous angular vel
         prev_ang_vel = m_net->outputs[1]->activation;
         //std:cout << "------------" << std::endl;
         //std::cout << mapped_ang_vel << std::endl;
         // std::cout << prev_ang_vel << std::endl;
         // std::cout << "--------------" << std::endl;

         //Set previous ranges
         range_tminus2 = range_tminus1;
         range_tminus1 = net_inputs[1];

      } else {

         leftSpeed = mapValueIntoRange(m_net->outputs[0]->activation,
                                             NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
                                             MIN_WHEEL_SPEED, MAX_WHEEL_SPEED);

         rightSpeed = mapValueIntoRange(m_net->outputs[1]->activation,
                                             NET_OUTPUT_LOWER_BOUND, NET_OUTPUT_UPPER_BOUND,
                                             MIN_WHEEL_SPEED, MAX_WHEEL_SPEED);

      }

      //leftSpeed = 10.0;
      //rightSpeed = 10.0;
      //std::cout << "\n" << leftSpeed << " " << rightSpeed << std::endl;
      //std::cout << "------------" << std::endl;

      m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);

    }



    //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    //cout << time_span.count() << std::endl;

}


double CArgosRosBotNEAT::mapHorizontalAngle(double angle) {

   return abs(1/M_PI * angle);

}

void CArgosRosBotNEAT::ConvertLinVelToWheelSpeed(Real linear_x, Real angular_z) {

  Real v = linear_x;// Forward speed
  Real w = angular_z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = v - HALF_BASELINE * w;
  rightSpeed = v + HALF_BASELINE * w;

  //stepsSinceCallback = 0;
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
   
   delete neatOrg;
   //Read in new genome

   std::ifstream iFile ("ibug_working_directory/temp/temp_gnome");
   //std::ifstream iFile ("ibug_working_directory/temp/footbot_start_24");    //Nothing to do with file

   char curword[20];
   int id;

   iFile >> curword;
   iFile >> id;

   NEAT::Genome *start_genome = new NEAT::Genome(id,iFile);
   iFile.close();

   neatOrg = new NEAT::Organism(0.0,start_genome,1);
   m_net = neatOrg->net;

   //delete start_genome;

   net_inputs[0] = 1.0;                            //Bias node

   //Initialise inputs
   // for(int i = 1; i < m_net->inputs.size(); i++) {
   //
   //     net_inputs[i] = 1.0;
   //     net_outputs[i] = 1.0;
   //
   // }

   prev_ang_vel = 0;

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
