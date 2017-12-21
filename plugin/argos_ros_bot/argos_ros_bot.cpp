// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <sstream>
#include <string>
#include <iostream>
#include <vector>       // std::vector


#include <ros/callback_queue.h>

#include <math.h>
#include "tf/LinearMath/Transform.h"


#include "argos_bridge/GetCmds.h"


using namespace std;
using namespace argos_bridge;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosBot::nodeHandle = initROS();

/****************************************/
/****************************************/

CArgosRosBot::CArgosRosBot() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcOmniCam(NULL),
  m_pcRangeBearing(NULL),
//  m_pcGripper(NULL),
  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  globalSteps(0),
  leftSpeed(0),
  rightSpeed(0)//,
//  gripping(false)
{
}

void CArgosRosBot::Init(TConfigurationNode& t_node) {
  // Create the topics to publish
  stringstream puckListTopic, proximityTopic, rangebearingTopic, poseTopic;
  puckListTopic << "/" << GetId() << "/puck_list";
  proximityTopic << "/" << GetId() << "/proximity";
  rangebearingTopic << "/" << GetId() << "/rangebearing";
  poseTopic << "/" << GetId() << "/position";
  puckListPub = nodeHandle->advertise<PuckList>(puckListTopic.str(), 1000);
  proximityPub = nodeHandle->advertise<ProximityList>(proximityTopic.str(), 1000);
  rangebearingPub = nodeHandle->advertise<RangebearingList>(rangebearingTopic.str(), 1000);
  posePub = nodeHandle->advertise<geometry_msgs::PoseStamped>(poseTopic.str(), 1000);


  // Create the subscribers
  stringstream cmdVelTopic;//, gripperTopic;

  cmdVelTopic << "/" << GetId() << "/cmd_vel";
//  gripperTopic << "/" << GetId() << "/gripper";
  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 10, &CArgosRosBot::cmdVelCallback, this);
//  gripperSub = nodeHandle->subscribe(gripperTopic.str(), 1, &CArgosRosBot::gripperCallback, this);


  client = nodeHandle->serviceClient<argos_bridge::GetCmds>("/bot0/get_vel_cmd");

  // Create the subscribers
//, gripperTopic;
  for(int n=0;n<NUMOFBOTS;n++)
  {
	  stringstream otherBotPoseTopic;
  otherBotPoseTopic << "/bot" << to_string(n) << "/position";
  cout<<otherBotPoseTopic.str()<<endl;

//  gripperTopic << "/" << GetId() << "/gripper";
  otherBotSub[n] = nodeHandle->subscribe(otherBotPoseTopic.str(), 1, &CArgosRosBot::otherBotPoseCallback, this);
  }

//  gripperSub = nodeHandle->subscribe(gripperTopic.str(), 1, &CArgosRosBot::gripperCallback, this);


  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcOmniCam = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
//  m_pcGripper = GetActuator<CCI_FootBotGripperActuator>("footbot_gripper");

  m_pcOmniCam->Enable();


  Rangebearing Rab;
  Rab.angle = 0.0f;
  Rab.range = 0.0f;

  RabList.n = NUMOFBOTS-1;
  for(int i = 0; i<NUMOFBOTS-1;i++)
	  RabList.Rangebearings.push_back(Rab);


  first_run = true;
  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}


void CArgosRosBot::Reset()
{
  first_run = true;

}

// Compares pucks for sorting purposes.  We sort by angle.
bool puckComparator(Puck a, Puck b) {
  return a.angle < b.angle;
}

bool cmd_is_new = false;
void CArgosRosBot::ControlStep() {



  const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcOmniCam->GetReadings();
  PuckList puckList;
  puckList.n = camReads.BlobList.size();
  for (size_t i = 0; i < puckList.n; ++i) {
    Puck puck;
    puck.type = (camReads.BlobList[i]->Color == CColor::RED);
    puck.range = camReads.BlobList[i]->Distance;
    // Make the angle of the puck in the range [-PI, PI].  This is useful for
    // tasks such as homing in on a puck using a simple controller based on
    // the sign of this angle.
    puck.angle = camReads.BlobList[i]->Angle.SignedNormalize().GetValue();
    puckList.pucks.push_back(puck);
  }

  // Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
  // the local puck configuration (e.g. fitting a lines to the detected pucks).
  sort(puckList.pucks.begin(), puckList.pucks.end(), puckComparator);

  puckListPub.publish(puckList);

  /* Get readings from proximity sensor */

  const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  proxList.header.seq = globalSteps;
  for (size_t i = 0; i < proxList.n; ++i) {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);


//cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
  }


   //Get readings from range and bearing sensor
   const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRangeBearing->GetReadings();
   RangebearingList RabList;
   RabList.Rangebearings.resize(tRabReads.size());
   RabList.n = tRabReads.size();
   for (size_t i = 0; i < RabList.n; ++i) {
      Rangebearing Rab;
      Rab.range = tRabReads[i].Range;
      Rab.angle = tRabReads[i].HorizontalBearing.GetValue();
      RabList.Rangebearings.at(i)=Rab;
   }


   /*Read out position of bot*/
   const CCI_PositioningSensor::SReading& sPosRead = m_pcPositioning->GetReading();
   PosQuat.header.frame_id = GetId();
   PosQuat.pose.position.x = sPosRead.Position.GetX();
   PosQuat.pose.position.y = sPosRead.Position.GetY();
   PosQuat.pose.position.z = sPosRead.Position.GetZ();
   PosQuat.pose.orientation.x = sPosRead.Orientation.GetX();
   PosQuat.pose.orientation.y = sPosRead.Orientation.GetY();
   PosQuat.pose.orientation.z = sPosRead.Orientation.GetZ();
   PosQuat.pose.orientation.w = sPosRead.Orientation.GetW();


/*   proximityPub.publish(proxList);
   rangebearingPub.publish(RabList);*/
   posePub.publish(PosQuat);

   if(GetId()!="bot1")
   {

   argos_bridge::GetCmds srv;


   srv.request.RabList = RabList;
   srv.request.PosQuat = PosQuat;
   srv.request.proxList = proxList;
   if(first_run)
   {
   srv.request.reset = true;
   first_run = false;
   }
   else
     srv.request.reset = false;


   client.call(srv);
   cmdVelCallback(srv.response.cmd_vel);

  // Wait for any callbacks to be called.

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount) {
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    stepsSinceCallback++;
  }

 /* if(cmd_is_new)
  {*/

  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
/*
  cmd_is_new = false;
  }else
    m_pcWheels->SetLinearVelocity(0, 0);
*/
  globalSteps ++;


  }
}



void CArgosRosBot::otherBotPoseCallback(const geometry_msgs::PoseStamped& pose){

	//geometry_msgs::PoseStamped PosQuat;
	const CCI_PositioningSensor::SReading& sPosRead = m_pcPositioning->GetReading();

	//Extract number from name
	int currentId, otherId;
	char c;
	std::stringstream iss1(GetId());
	iss1 >>  c >> c >> c >> currentId;
	std::stringstream iss2(pose.header.frame_id);
	iss2  >> c >> c >> c >> otherId;

	// If the current ID is not the same the ID from the received message
	if(otherId!=currentId)
	{
		//Get the difference in poses to get the range
		tf::Quaternion q_current(PosQuat.pose.orientation.x,
				PosQuat.pose.orientation.y,
				PosQuat.pose.orientation.z,
				PosQuat.pose.orientation.w);
		tf::Vector3 p_current(PosQuat.pose.position.x,
				PosQuat.pose.position.y,
				PosQuat.pose.position.z);
		tf::Transform tf_current(q_current,p_current);
		tf::Quaternion q_other(pose.pose.orientation.x,
				pose.pose.orientation.y,
				pose.pose.orientation.z,
				pose.pose.orientation.w);
		tf::Vector3 p_other(pose.pose.position.x,
				pose.pose.position.y,
				pose.pose.position.z);
		tf::Transform tf_other(q_other,p_other);
		tf::Transform tf_diff = tf_other.inverseTimes(tf_current);
		double range = tf_diff.getOrigin().length();

		//Get the bearing to the other bot
		double yaw_current = tf::getYaw(tf_current.getRotation());
		// Get position and change to range and angle
		double x =  PosQuat.pose.position.x;
		double y =  PosQuat.pose.position.y;
		double angle = std::atan2(((double)pose.pose.position.y -y),((double)pose.pose.position.x -x));
		double bearing=angle-yaw_current;


		//Fil up the range bearing list  msg
		//TODO: Right now this is not bearing but absolute angle!!
		// Change to bearing by taking it relative from the current ID's angle

		if(otherId<currentId)
		{

			RabList.Rangebearings[otherId].range=range;
			RabList.Rangebearings[otherId].angle=bearing;
		} else {
			RabList.Rangebearings[otherId-1].range=range;
			RabList.Rangebearings[otherId-1].angle=bearing;
		}

		//rangebearingPub.publish(RabList);
	}
}

void CArgosRosBot::cmdVelCallback(const geometry_msgs::Twist& twist) {
 // cout << "cmdVelCallback: " << GetId() << endl;

  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.

 /* srand (time(NULL));



  float v1 = (float)(rand() % 10 + 95)/ 100;
  float v2 = (float)(rand() % 10 + 95) / 100;
*/
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
  cmd_is_new =true;
}

/*
void CArgosRosBot::gripperCallback(const std_msgs::Bool& value) {
  cout << "gripperCallback: " << GetId() << endl;

  if (gripping && !value.data) {
    // Release the gripper
    m_pcGripper->Unlock();
    gripping = true;
  }

  if (!gripping && value.data) {
    // Activate gripper
    m_pcGripper->LockPositive();
    gripping = false;
  }

  stepsSinceCallback = 0;
}
*/

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
REGISTER_CONTROLLER(CArgosRosBot, "argos_ros_bot_controller")
