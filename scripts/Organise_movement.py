# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 16:43:20 2021

@author: ep15603
"""

"""

Organise movement node 

"""
#!/usr/bin/env python

from str_msgs.msg import String, Float32
from geometry_msgs.msg import Pose
import rospy
import json


class OrganizeMovement:
    def __init__(self):

        rospy.init_node("organize_movement")

        # self.mode = None
        # self.base = False
        # self.arm = False
        self.goal = None
        self.trophy_info = None

        self.pub_base = rospy.Publisher('/base_cntrl/go_to_pose', Pose, queue_size=3)
        self.pub_arm = rospy.Publisher('/arm_cmd', Float32, queue_size=3)
        self.pub_gripper = rospy.Publisher('/start_gripper', String, queue_size=3)

        self.sub_cmd = rospy.Subscriber("/target_move", String, self.on_activation)
        self.sub_base = rospy.Subscriber("/base_cntrl/out_result", String, self.base_in_position)
        self.sub_arm = rospy.Subscriber("/arm_result", String, self.arm_in_position)
        self.sub_trophy = rospy.Subscriber("/trophy_list", String, self.trophy_update)

    def on_activation(self, msg):
        command = json.loads(msg.data)
        self.goal = command["id"]
        self.trophy_info = command["description"]
        base_position = (float(self.trophy_info["x"]), float(self.trophy_info["y"]), float(self.trophy_info["alpha"]))
        self.pub_base.publish(base_position)

    def base_in_position(self, msg):
        self.pub_arm.publish(float(self.trophy_info["z"]))

    def arm_in_position(self, msg):
        self.pub_gripper.publish("grip")

    def trophy_update(self, msg):
        # TODO: If the goal trophy is at a significantly different place than currently thought: update
        # TODO: If the goal trophy does not exist: abort
        trophy_list = json.loads(msg.data)
        try:
            info_update = trophy_list[self.goal]
        except KeyError:
            # Abort
            pass


if __name__ == '__main__':
    try:
        OrganizeMovement()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("OrganizeMovement terminating.")
