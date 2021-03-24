#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Sat Mar 20 15:43:04 2021

@author: ep15603
"""
""" 

Organise movement node 

"""

from std_msgs.msg import String, Float32
import rospy
import json


class MoveGripper:
    def __init__(self):

        rospy.init_node("move_gripper")

        self.pub_gripper = rospy.Publisher('/gripper_cmd', String, queue_size=3)
        self.pub_restart = rospy.Publisher('/restart_search', String, queue_size=3)
        self.pub_return = rospy.Publisher('/return_cmd', String, queue_size=3)

        self.sub_move = rospy.Subscriber("/start_gripper", String, self.on_activation)
        self.sub_gripper = rospy.Subscriber("/gripper_result", String, self.gripper_in_position)
        self.sub_trophy = rospy.Subscriber("/trophy_list", String, self.trophy_update)

    def on_activation(self, msg):
        goal = msg.data
        if goal == "grip":
            self.pub_gripper.publish("grip")
        else:
            self.pub_gripper.publish("drop")

    def gripper_in_position(self, msg):
        result = msg.data
        if result == "success":
            self.pub_return.publish("return")
        else:
            self.pub_restart.publish("restart")

    def trophy_update(self, msg):
        # trophy_list = json.loads(msg.data)
        pass


if __name__ == '__main__':
    try:
        MoveGripper()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveGripper terminating.")
