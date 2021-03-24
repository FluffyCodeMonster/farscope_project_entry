#!/usr/bin/env python

from std_msgs.msg import String, Float32, Int16
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
#import tf_conversions as tf_conv
import rospy
import json
import numpy as np

class OrganizeMovement:
    def __init__(self):

        rospy.init_node("organize_movement")

        # self.mode = None
        # self.base = False
        # self.arm = False
        self.goal = None
        self.trophy_info = None

        self.pub_base = rospy.Publisher('/base_cntrl/go_to_pose', Pose, queue_size=3)
        self.pub_arm = rospy.Publisher('/arm_cmd', Int16, queue_size=3)
        self.pub_gripper = rospy.Publisher('/start_gripper', String, queue_size=3)

        self.sub_cmd = rospy.Subscriber("/target_move", String, self.on_activation)
        self.sub_base = rospy.Subscriber("/base_cntrl/out_result", String, self.base_in_position)
        self.sub_arm = rospy.Subscriber("/arm_result", String, self.arm_in_position)
        self.sub_trophy = rospy.Subscriber("/trophy_list", String, self.trophy_update)

    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def on_activation(self, msg):
        command = json.loads(msg.data)
        self.goal = command["id"]
        self.trophy_info = command["description"]
        #quat = tf_conv.transformations.quaternion_from_euler(0, 0, float(self.trophy_info["alpha"]), axes='sxyz')
        quat = self.euler_to_quaternion(0, 0, float(self.trophy_info["alpha"]))
        pose = (Pose(Point(float(self.trophy_info["x"]),
                           float(self.trophy_info["y"]), 0.0), Quaternion(quat[0], quat[1], quat[2], quat[3])))
        self.pub_base.publish(pose)

    def base_in_position(self, msg):
        self.pub_arm.publish(int(self.trophy_info["z"]))

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
