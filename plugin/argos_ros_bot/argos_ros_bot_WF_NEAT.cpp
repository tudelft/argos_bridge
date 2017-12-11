// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot_WF_NEAT.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>


#include <sstream>
#include <string>
#include <iostream>

#include <ros/callback_queue.h>

#include <math.h>
#include "tf/LinearMath/Transform.h"


using namespace std;

/****************************************/
/****************************************/

CArgosRosBotWFNEAT::CArgosRosBotWFNEAT() :
              m_pcWheels(NULL),
              m_pcProximity(NULL),
              m_pcRangeBearing(NULL),
              m_pcPositioning(NULL),
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
              argosTime(0),
              stateFinished(true)
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

void CArgosRosBotWFNEAT::Init(TConfigurationNode& t_node) {

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");


  stateData.state = SStateData::STATE_FORWARD;

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}

void CArgosRosBotWFNEAT::ControlStep() {

  if(GetId()=="bot0")
  {
    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

    // Get readings from range and bearing sensor
    const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRangeBearing->GetReadings();


    const CCI_PositioningSensor::SReading& sPosRead = m_pcPositioning->GetReading();

    CArgosRosBotWFNEAT::StateMachine(tProxReads,tRabReads,sPosRead);


    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
    argosTime++;

  }


}

void CArgosRosBotWFNEAT::StateMachine(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{

  CArgosRosBotWFNEAT::StateHandlingNEAT(tProxReads,tRabReads,sPosRead);
  CArgosRosBotWFNEAT::StateHandling(tProxReads,tRabReads,sPosRead);
  CArgosRosBotWFNEAT::StateActions(tProxReads,tRabReads,sPosRead);


}


bool leave_wall;
bool stop_motion;
bool go_around_wall;

void CArgosRosBotWFNEAT::StateHandlingNEAT(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{
  //Include inputs for both the range and the bearing
  for(int i = 0; i < tRabReads.size(); i++) {
    net_inputs[(i*2)+1] = mapValueIntoRange(tRabReads[i].Range,
        RANGE_SENSOR_LOWER_BOUND, RANGE_SENSOR_UPPER_BOUND,
        NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
    //   net_inputs[(i*2)+2] = mapValueIntoRange(tRabReads[i].HorizontalBearing.GetValue(),
    //                                       BEARING_SENSOR_LOWER_BOUND, BEARING_SENSOR_UPPER_BOUND,
    //                                       NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
    net_inputs[(i*2)+2] =CArgosRosBotWFNEAT::mapHorizontalAngle(tRabReads[i].HorizontalBearing.GetValue());
  }

  //std::cout << net_inputs.size() << std::endl;

  //Proximity sensor inputs
  for(int i = 0; i < tProxReads.size(); i++) {
    net_inputs[i+(tRabReads.size()*2)+1] = CArgosRosBotWFNEAT::mapValueIntoRange(tProxReads[i].Value,
        PROX_SENSOR_LOWER_BOUND, PROX_SENSOR_UPPER_BOUND,
        NET_INPUT_LOWER_BOUND, NET_INPUT_UPPER_BOUND);
  }

  // State input
  for(int i = 0; i < 5; i++) {

    net_inputs[i+(tRabReads.size()*2)+tProxReads.size()+1]=0;

  }
  net_inputs[stateData.state+(tRabReads.size()*2)+tProxReads.size()+1]=1;

  /*  for(int i = 0; i<net_inputs.size();i++)
    std::cout<<net_inputs[i]<<" ";

  std::cout<<" "<<std::endl;*/


  m_net->load_sensors(net_inputs);

  if (!(m_net->activate())) std::cout << "Inputs disconnected from output!";

/*

  for(int i = 0; i< m_net->outputs.size();i++)
    std::cout<<m_net->outputs[i]->activation<<" ";


*/






  int maxPos = 0;
  for (unsigned i = 0; i < m_net->outputs.size(); ++i)
  {

    if (m_net->outputs[i]->activation > m_net->outputs[maxPos]->activation ) // Found a bigger max
      maxPos = i;
  }




  leave_wall = false;
  stop_motion = false;
  go_around_wall = true;
  switch(maxPos){
    case 0:
      leave_wall = true;
    //  std::cout<<"LEAVE WALL"<<std::endl;
      break;
    case 1:
     // std::cout<<"STOP MOTION"<<std::endl;
      stop_motion = true;
      break;
    case 2:
      stop_motion = true;

      break;
    case 3:
     // go_around_wall=true; // remove for full wall following

      break;
    case 4:

      break;
  }


}

void CArgosRosBotWFNEAT::StateHandling(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{
  Real tower_bearing = tRabReads[0].HorizontalBearing.GetValue();

  CRadians cTmp1, cTmp2, cOrientationZ;

  sPosRead.Orientation.ToEulerAngles(cOrientationZ, cTmp1, cTmp2);

  Real range_10 = tProxReads[10].Value;
  Real range_8 = tProxReads[8].Value;
  Real range_9 = tProxReads[8].Value;

  Real range_0 = tProxReads[0].Value;
  Real range_11 = tProxReads[11].Value;
  Real range_1 = tProxReads[1].Value;


  if (range_10 == 0.0f) range_10 = 1000.0f;
  if (range_9 == 0.0f) range_9 = 1000.0f;

  if (range_8 == 0.0f) range_8 = 1000.0f;
  if (range_1 == 0.0f) range_1 = 1000.0f;
  if (range_0 == 0.0f) range_0 = 1000.0f;
  if (range_11 == 0.0f) range_11 = 1000.0f;



  // Handle state transactions
  switch(stateData.state){
    case  SStateData::STATE_FORWARD:
    {
      if(range_0< 0.07 || range_1 < 0.07 || range_11< 0.07){
        stateData.state = SStateData::STATE_STOP;
        stateData.timeSinceStateChange = argosTime;
        stateData.headingSinceStateChange = cOrientationZ;
      }
      if(leave_wall)
      {
        stateData.state = SStateData::STATE_ROTATE_TO_GOAL;
      }
      if(stop_motion)
        stateData.state = SStateData::STATE_STOP;

      break;
    }
    case  SStateData::STATE_ROTATE_AROUND_WALL:
    {
      if(range_0< 0.07 || range_1 < 0.07 || range_11< 0.07){
        stateData.state = SStateData::STATE_STOP;
        stateData.timeSinceStateChange = argosTime;
        stateData.headingSinceStateChange = cOrientationZ;
      }
      if(leave_wall)
      {
        stateData.state = SStateData::STATE_ROTATE_TO_GOAL;
      }
      break;
    }
    case  SStateData::STATE_STOP:
    {
      if(argosTime-stateData.timeSinceStateChange>20 && stop_motion == false )
      {
        stateData.state = SStateData::STATE_TURN_FROM_WALL;
        stateData.timeSinceStateChange = argosTime;
        stateData.headingSinceStateChange = cOrientationZ;
      }
      break;
    }
    case  SStateData::STATE_TURN_FROM_WALL:
    {
      if(range_0>0.1 && range_1>0.1&&range_11>0.1 &&  range_9< 0.1 && range_8 < 0.1 && range_10< 0.1 && fabs(range_8-range_10)<0.01)
      {
        //  std::cout<<range_0<<" "<<range_1<<" "<<range_11<<" "<<range_9<<" "<<range_8<<" "<<range_10<<std::endl;
        stateData.state = SStateData::STATE_WALL_FOLLOWING;
        stateData.timeSinceStateChange = argosTime;
        stateData.headingSinceStateChange = cOrientationZ;
      }
      break;
    }
    case SStateData::STATE_WALL_FOLLOWING:
    {
      if(range_0< 0.07 || range_1 < 0.07 || range_11< 0.07){
        stateData.state = SStateData::STATE_STOP;
        stateData.timeSinceStateChange = argosTime;
        stateData.headingSinceStateChange = cOrientationZ;
      }
      if(leave_wall)
      {
        stateData.state = SStateData::STATE_ROTATE_TO_GOAL;
      }
      if(range_10>0.1)
        if(go_around_wall)
          stateData.state = SStateData::STATE_ROTATE_AROUND_WALL;
        else
          stateData.state = SStateData::STATE_FORWARD;
      break;
    }
    case SStateData::STATE_ROTATE_TO_GOAL:
    {
      if(tower_bearing<0.1&&tower_bearing>-0.1){
        stateData.state = SStateData::STATE_FORWARD;
        stateData.timeSinceStateChange = argosTime;
        stateData.headingSinceStateChange = cOrientationZ;
      }
      break;
    }

  }
}


void CArgosRosBotWFNEAT::StateActions(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{

  Real range_10 = tProxReads[10].Value;
  Real range_8 = tProxReads[8].Value;



  if (range_10 == 0.0f) range_10 = 1000.0f;
  if (range_8 == 0.0f) range_8 = 1000.0f;

  double turn_rate = -1.0*0.2/0.3;

  // Handle actions per state
  switch(stateData.state){
    case  SStateData::STATE_FORWARD:
      ConvertDifferentialDriveToSpeed(0.2,0);
     //  std::cout<<"forward"<<std::endl;
      break;
    case  SStateData::STATE_ROTATE_AROUND_WALL:
      ConvertDifferentialDriveToSpeed(0.2,turn_rate);
     // std::cout<<"rotate around wall"<<std::endl;
      break;
    case SStateData::STATE_STOP:
      ConvertDifferentialDriveToSpeed(0,0);
    //   std::cout<<"stop"<<std::endl;
      break;
    case SStateData::STATE_TURN_FROM_WALL:
      ConvertDifferentialDriveToSpeed(0,1);
    // std::cout<<"turn"<<std::endl;

      break;
    case SStateData::STATE_WALL_FOLLOWING:
      //  std::cout<<"wall_following"<<std::endl;
      if(range_10>range_8)
        ConvertDifferentialDriveToSpeed(0.2,-0.1);
      if(range_10<range_8)
        ConvertDifferentialDriveToSpeed(0.2,0.1);
      break;
    case SStateData::STATE_ROTATE_TO_GOAL:
      // std::cout<<"wall_following"<<std::endl;
      ConvertDifferentialDriveToSpeed(0,0.5);
      /// std::cout<<"rotate to goal"<<std::endl;
      break;
  }
}


double CArgosRosBotWFNEAT::mapHorizontalAngle(double angle) {

  return abs(1/M_PI * angle);

}



void CArgosRosBotWFNEAT::ConvertDifferentialDriveToSpeed(Real linear_x, Real angular_z) {

  Real v = linear_x * 100;// Forward speed
  Real w = angular_z * 100; // Rotational speed

  //34.34 with lin_vel = 1.0 and angular_vel = 0.0

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = v - HALF_BASELINE * w;
  rightSpeed = v + HALF_BASELINE * w;

  stepsSinceCallback = 0;
}

double CArgosRosBotWFNEAT::mapValueIntoRange(const double input, const double input_start,
    const double input_end, const double output_start,
    const double output_end) {

  if(input > input_end) return 1.0;

  double slope = (output_end - output_start) / (input_end - input_start);
  double output = output_start + slope * (input - input_start);

  return output;

}
void CArgosRosBotWFNEAT::Reset() {
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

  stateData.state = SStateData::STATE_FORWARD;


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
REGISTER_CONTROLLER(CArgosRosBotWFNEAT, "argos_ros_bot_wf_neat_controller")
