/*
 * AUTHOR: Andrew Vardy <av@mun.ca>
 *
 * Connects an ARGoS robot with a particular configuration to ROS by publishing
 * sensor values and subscribing to a desired wheel speeds topic.
 *
 */

#ifndef ARGOS_ROS_BOT_NEAT_H
#define ARGOS_ROS_BOT_NEAT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_gripper_actuator.h>


#include "../../ext_lib/NEAT/include/population.h"
#include "../../ext_lib/NEAT/include/network.h"



#include <string>


using namespace argos;

#define NUMOFBOTS 2

class CArgosRosBotWFNEAT : public CCI_Controller {

public:

  CArgosRosBotWFNEAT();
  virtual ~CArgosRosBotWFNEAT() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset();

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {};

  void ConvertDifferentialDriveToSpeed(Real linear_x, Real angular_z);
  void StateMachine(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
      const CCI_PositioningSensor::SReading& sPosRead);
  void StateHandling(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
      const CCI_PositioningSensor::SReading& sPosRead);
  void StateActions(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
      const CCI_PositioningSensor::SReading& sPosRead);
  void StateHandlingNEAT(const CCI_FootBotProximitySensor::TReadings& tProxReads, const CCI_RangeAndBearingSensor::TReadings& tRabReads,
      const CCI_PositioningSensor::SReading& sPosRead);
  double mapValueIntoRange(const double input, const double input_start,
                                             const double input_end, const double output_start,
                                             const double output_end);
  double mapHorizontalAngle(double angle);

  /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      enum EState {
         STATE_FORWARD,
         STATE_STOP,
         STATE_TURN_FROM_WALL,
         STATE_WALL_FOLLOWING,
         STATE_ROTATE_TO_GOAL,
         STATE_ROTATE_AROUND_WALL,
      } state;
      int timeSinceStateChange;
      CRadians headingSinceStateChange;
   };

private:


   const double NET_INPUT_LOWER_BOUND;
   const double NET_INPUT_UPPER_BOUND;
   const double RANGE_SENSOR_LOWER_BOUND;
   const double RANGE_SENSOR_UPPER_BOUND;
   const double PROX_SENSOR_LOWER_BOUND;
   const double PROX_SENSOR_UPPER_BOUND;
   const double BEARING_SENSOR_LOWER_BOUND;
   const double BEARING_SENSOR_UPPER_BOUND;

   const double NET_OUTPUT_LOWER_BOUND;
   const double NET_OUTPUT_UPPER_BOUND;
   const double MIN_LINEAR_VEL;
   const double MAX_LINEAR_VEL;
   const double MIN_ANGULAR_VEL;
   const double MAX_ANGULAR_VEL;
   const double MAX_WHEEL_SPEED;
   const double MIN_WHEEL_SPEED;


  CCI_DifferentialSteeringActuator* m_pcWheels;
  CCI_FootBotProximitySensor* m_pcProximity;
  CCI_RangeAndBearingSensor* m_pcRangeBearing;
  CCI_PositioningSensor* m_pcPositioning;

  // The following constant values were copied from the argos source tree from
  // the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
  static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
  //static constexpr Real WHEEL_RADIUS = 0.029112741f;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><argos_ros_bot_controller> section.
   */

  // The number of time steps from the time step of the last callback
  // after which leftSpeed and rightSpeed will be set to zero.  Useful to
  // shutdown the robot after the controlling code on the ROS side has quit.
  int stopWithoutSubscriberCount;

  // The number of time steps since the last callback.
  int stepsSinceCallback;

  // Most recent left and right wheel speeds, converted from the ROS twist
  // message.
  Real leftSpeed, rightSpeed;

  SStateData stateData;
  bool stateFinished;

  int argosTime;

  NEAT::Network* m_net;

  std::vector<float> net_inputs;
  std::vector<double> net_outputs;

  int num_inputs;


};

#endif
