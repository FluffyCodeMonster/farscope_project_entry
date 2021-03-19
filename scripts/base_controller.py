#!/usr/bin/env python
import sys, rospy
from std_msgs.msg import String
from farscope_group_project.farscope_robot_utils import BaseDriver

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class BaseController:
    def __init__(self):
        # This will be our node name
        rospy.init_node("base_controller")

        # We will subscribe to a String command topic
        self.base_sub = rospy.Subscriber("/base_cntrl/in_cmd", String, self.on_command)

        # We will publish a String feedback topic
        self.base_pub = rospy.Publisher("/base_cntrl/out_result", String, queue_size=3)

        # Base driver object to allow for a simple way to request movement from the actual robot
        self.base_driver = BaseDriver()

        ## Publisher to manually control the robot (e.g. to stop it)
        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        ## Subscribe to the move_base action server
        #self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)


        # wait for parameter that says spawning is finished
        rospy.loginfo("Waiting for spawning to finish")
        rate = rospy.Rate(2.0)
        while not rospy.get_param('target_spawning_complete',False):
            rate.sleep()
        rospy.loginfo("Spawning completion detected - time to get going!")
        
        # Clean up on shutdown        
        rospy.on_shutdown(self.shutdown)

    # This is how we'll react on the commands received
    def on_command(self, cmd):
        if cmd.data == "fwd":
            self.base_driver.move(1.0)
            self.base_pub.publish("OK FWD")
        elif cmd.data == "back":
            self.base_driver.move(-1.0)
            self.base_pub.publish("OK BACK")
        elif cmd.data == "right":
            self.base_driver.move(0, -1.0)
            self.base_pub.publish("OK RIGHT")
        elif cmd.data == "left":
            self.base_driver.move(0, 1.0)
            self.base_pub.publish("OK LEFT")
        elif cmd.data == "spin":
            self.base_driver.move(0, 0, 0.1, 50)
            self.base_pub.publish("OK SPIN")    
    
    def shutdown(self):
        rospy.loginfo("Stopping base controller...")
        # Cancel any active goals
        #self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
if __name__ == '__main__':
    try:
        BaseController()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Base Controller terminating.")
