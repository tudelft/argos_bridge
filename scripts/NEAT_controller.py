#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
import math, random
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class NEATController:

    cmdVelPub = None
    time = 0
    stateStartTime = 0
    lastTwist = None
    

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('NEAT_outputs', Float64MultiArray, self.neat_callback)
        
    def neat_callback(self, net_outputs):
        
        twist = Twist()
        twist.linear.x = net_outputs.data[0]
        twist.angular.z = net_outputs.data[1]
        
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist

if __name__ == '__main__':
    rospy.init_node("NEAT_controller")
    controller = NEATController()
    rospy.spin()
