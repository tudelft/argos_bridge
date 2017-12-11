// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot_WF.h"
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

CArgosRosBotWF::CArgosRosBotWF() :
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
      argosTime(0)
{

}

void CArgosRosBotWF::Init(TConfigurationNode& t_node) {

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");


  stateData.state = SStateData::STATE_ROTATE_TO_GOAL;

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}

void CArgosRosBotWF::ControlStep() {

  if(GetId()=="bot0")
    {
      /* Get readings from proximity sensor */
      const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

      // Get readings from range and bearing sensor
      const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRangeBearing->GetReadings();


      const CCI_PositioningSensor::SReading& sPosRead = m_pcPositioning->GetReading();

      CArgosRosBotWF::StateMachine(tProxReads,tRabReads,sPosRead);


      m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
      argosTime++;

    }


}

void CArgosRosBotWF::StateMachine(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{

  CArgosRosBotWF::StateHandling(tProxReads,tRabReads,sPosRead);
  CArgosRosBotWF::StateActions(tProxReads,tRabReads,sPosRead);


}


void CArgosRosBotWF::StateHandling(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{
  Real tower_bearing = tRabReads[0].HorizontalBearing.GetValue();

        CRadians cTmp1, cTmp2, cOrientationZ;

        sPosRead.Orientation.ToEulerAngles(cOrientationZ, cTmp1, cTmp2);

        Real range_10 = tProxReads[10].Value;
        Real range_8 = tProxReads[8].Value;

        Real range_0 = tProxReads[0].Value;
        Real range_11 = tProxReads[11].Value;
        Real range_1 = tProxReads[1].Value;


        if (range_10 == 0.0f) range_10 = 1000.0f;
        if (range_8 == 0.0f) range_8 = 1000.0f;
        if (range_1 == 0.0f) range_1 = 1000.0f;
        if (range_0 == 0.0f) range_0 = 1000.0f;
        if (range_11 == 0.0f) range_11 = 1000.0f;


        // Handle state transactions
        switch(stateData.state){
          case  SStateData::STATE_FORWARD:
          {
            if(range_0< 0.7 && range_1 < 1 && range_11< 1){
              stateData.state = SStateData::STATE_STOP;
              stateData.timeSinceStateChange = argosTime;
              stateData.headingSinceStateChange = cOrientationZ;
            }
            break;

          case  SStateData::STATE_STOP:
          {
            if(argosTime-stateData.timeSinceStateChange>20)
            {
              stateData.state = SStateData::STATE_TURN_FROM_WALL;
              stateData.timeSinceStateChange = argosTime;
              stateData.headingSinceStateChange = cOrientationZ;
            }
            break;
          }
          case  SStateData::STATE_TURN_FROM_WALL:
          {
            if(argos::NormalizedDifference( cOrientationZ,stateData.headingSinceStateChange).GetValue()>1.48)
            {
              stateData.state = SStateData::STATE_WALL_FOLLOWING;
              stateData.timeSinceStateChange = argosTime;
              stateData.headingSinceStateChange = cOrientationZ;
            }
            break;
          }
          case SStateData::STATE_WALL_FOLLOWING:
          {
            if(range_0< 0.7 && range_1 < 1 && range_11< 1){
              stateData.state = SStateData::STATE_STOP;
              stateData.timeSinceStateChange = argosTime;
              stateData.headingSinceStateChange = cOrientationZ;
            }
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
}

void CArgosRosBotWF::StateActions(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
    const CCI_PositioningSensor::SReading& sPosRead)
{

  Real range_10 = tProxReads[10].Value;
  Real range_8 = tProxReads[8].Value;



  if (range_10 == 0.0f) range_10 = 1000.0f;
  if (range_8 == 0.0f) range_8 = 1000.0f;


  // Handle actions per state
  switch(stateData.state){
    case  SStateData::STATE_FORWARD:
      ConvertDifferentialDriveToSpeed(0.2,0);
      std::cout<<"forward"<<std::endl;
      break;
    case SStateData::STATE_STOP:
      ConvertDifferentialDriveToSpeed(0,0);
      std::cout<<"stop"<<std::endl;
      break;
    case SStateData::STATE_TURN_FROM_WALL:
      ConvertDifferentialDriveToSpeed(0,1);
      std::cout<<"turn"<<std::endl;

      break;
    case SStateData::STATE_WALL_FOLLOWING:
      std::cout<<range_10<<" "<<range_8<<std::endl;
      if(range_10>range_8)
        ConvertDifferentialDriveToSpeed(0.2,-0.1);
      if(range_10<range_8)
        ConvertDifferentialDriveToSpeed(0.2,0.1);
       break;
    case SStateData::STATE_ROTATE_TO_GOAL:
      ConvertDifferentialDriveToSpeed(0,0.5);
      std::cout<<"rotate to goal"<<std::endl;
      break;
  }
}





void CArgosRosBotWF::ConvertDifferentialDriveToSpeed(Real linear_x, Real angular_z) {

  Real v = linear_x * 100;// Forward speed
  Real w = angular_z * 100; // Rotational speed

  //34.34 with lin_vel = 1.0 and angular_vel = 0.0

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = v - HALF_BASELINE * w;
  rightSpeed = v + HALF_BASELINE * w;

  stepsSinceCallback = 0;
}


void CArgosRosBotWF::Reset() {


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
REGISTER_CONTROLLER(CArgosRosBotWF, "argos_ros_bot_wf_controller")
