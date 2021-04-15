#!/usr/bin/env python

from std_msgs.msg import String, Float32, Int16
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
import rospy
import json
import numpy as np


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class Trophy:
    def __init__(self, tid, x, y, z, shelf, level, w):
        self.id = tid
        self.x = x
        self.y = y
        self.z = z
        self.shelf = shelf
        self.level = level
        self.w = w


class Strategy:
    def __init__(self):

        rospy.init_node("strategy")

        # TODO: Create file including the missing data
        # TODO: Load files here

        self.base_x = 0
        self.base_y = 0
        self.base_alpha = 0

        self.shelf_positions = None

        # Current Mode of Robot
        # 0 = Idle
        # 1 = Robot Moving
        # 2 = Arm Moving
        # 3 = Robot Gripping
        self.mode = 0

        # Current Phase of Strategy
        # 0 = Get Trophy
        # 1 = Return Trophy
        # 2 = Finished
        self.phase = 0

        # Current Target Trophy
        self.trophy_goal = None

        # List of all Trophies, Constantly Updated
        self.trophy_list = None
        self.trophy_map = None
        self.neighbor_score_mask = None
        self.update_trophy_map()

        self.pub_base = rospy.Publisher('/base_cntrl/go_to_pose', Pose, queue_size=3)
        self.pub_arm = rospy.Publisher('/arm_cmd', Int16, queue_size=3)
        self.pub_gripper = rospy.Publisher('/gripper_cmd', String, queue_size=3)

        self.sub_base = rospy.Subscriber("/base_cntrl/out_result", String, self.base_in_position)
        self.sub_arm = rospy.Subscriber("/arm_result", String, self.arm_in_position)
        self.sub_gripper = rospy.Subscriber("/gripper_result", String, self.gripper_in_position)
        self.sub_trophy = rospy.Subscriber("/trophy_update", String, self.trophy_update)

        # TODO: wait until everything is loaded (as implemented in base_controller.py)

        self.trophy_goal = self.score_one()
        self.move_base_to_goal()

    def update_trophy_map(self):
        self.trophy_map = np.zeros((3, 8))
        for trophy in self.trophy_list:
            self.trophy_map[trophy.level, trophy.shelf] = self.trophy_map[trophy.level, trophy.shelf] + 1

    def score_one(self):
        max_val = (0, None)
        for trophy in self.trophy_list:
            deploy_time = self.calculate_deploy_time(trophy)
            n_density = self.calculate_n_density(trophy)
            # info_gain = self.calculate_info_gain
            score = 1.0 * deploy_time + 0 * n_density
            if score > max_val[0]:
                max_val = (score, trophy)
        return max_val[1]

    def score_two(self):
        max_val = (0, None)
        for trophy in self.trophy_list:
            deploy_time = self.calculate_deploy_time(trophy)
            n_density = self.calculate_n_density(trophy)
            difficulty = self.calculate_difficulty(trophy)
            score = 1.0 * deploy_time + 0 * n_density
            if score > max_val[0]:
                max_val = (score, trophy)
        return max_val[1]

    def calculate_deploy_time(self, trophy):
        # TODO: Either load deploy time from file or request from path planning
        pass

    def calculate_n_density(self, trophy):
        # TODO: Calculate density based on the trophy_map and neighbor_score_mask
        pass

    def calculate_info_gain(self, trophy):
        # Relict, not expected to be implemented
        pass

    def calculate_difficulty(self, trophy):
        # TODO: Implement function calculating the difficulty
        pass

    def move_base(self, x, y, alpha):
        quaternion = euler_to_quaternion(0, 0, alpha)
        pose = (Pose(Point(x, y, 0.0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])))
        self.mode = 1
        self.pub_base.publish(pose)

    def move_base_to_goal(self):
        x, y, alpha = self.shelf_positions[self.trophy_goal.shelf]
        self.move_base(x, y, alpha)

    def return_base(self):
        self.move_base(self.base_x, self.base_y, self.base_alpha)

    def base_in_position(self, msg):
        self.pub_arm.publish(self.trophy_goal.z)

    def trophy_update(self, msg):
        # TODO: Create list of Trophy objects from input and compare it with current list
        # TODO: If the goal trophy is at a significantly different place than currently thought: update
        # TODO: If the goal trophy does not exist: abort
        trophy_list = json.loads(msg.data)
        try:
            info_update = trophy_list[self.goal]
        except KeyError:
            # Abort
            pass

    def arm_in_position(self, msg):
        if self.phase == 0:
            self.pub_gripper.publish("grip")
        else:
            self.pub_gripper.publish("drop")

    def gripper_in_position(self, msg):
        result = msg.data
        if self.phase == 0:
            if result == "success":
                self.phase = 1
                self.return_base()
            else:
                self.score_two()
                self.move_base_to_goal()


if __name__ == '__main__':
    try:
        Strategy()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Strategy terminating.")
