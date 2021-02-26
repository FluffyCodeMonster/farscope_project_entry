#!/usr/bin/env python
import sys, rospy
from std_msgs.msg import String
from farscope_group_project.farscope_robot_utils import BaseDriver

# This is how we'll react on the commands received
def on_command(cmd):
    if cmd.data == "fwd":
        base_driver.move(1.0)
        base_pub.publish("OK FWD")
    elif cmd.data == "back":
        base_driver.move(-1.0)
        base_pub.publish("OK BACK")
    elif cmd.data == "right":
        base_driver.move(0, -1.0)
        base_pub.publish("OK RIGHT")
    elif cmd.data == "left":
        base_driver.move(0, 1.0)
        base_pub.publish("OK LEFT")
    elif cmd.data == "spin":
        base_driver.move(0, 0, 0.5, 10)
        base_pub.publish("OK SPIN")

# This will be our node name
rospy.init_node("base_controller")

# We will subscribe to a String command topic
base_sub = rospy.Subscriber("/base_cntrl/input", String, on_command)

# We will publish a String feedback topic
base_pub = rospy.Publisher("/base_cntrl/output", String, queue_size=3)

# Base driver object to allow for a simple way to request movement from the actual robot
base_driver = BaseDriver()

# wait for parameter that says spawning is finished
rospy.loginfo("Waiting for spawning to finish")
rate = rospy.Rate(2.0)
while not rospy.get_param('target_spawning_complete',False):
    rate.sleep()
rospy.loginfo("Spawning completion detected - time to get going!")

while not rospy.is_shutdown():
    rospy.spin()
