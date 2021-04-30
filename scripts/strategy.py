#!/usr/bin/env python

from std_msgs.msg import String, Float32, Int16, Float32MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
import rospy
import json
import yaml
import sys
import numpy as np


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class Trophy:
    def __init__(self, trophy_id, x, y, z, shelf, level, w):
        self.trophy_id = trophy_id
        self.x = x
        self.y = y
        self.z = z
        self.shelf = shelf
        self.level = level
        self.w = w


class Strategy:
    def __init__(self, scenario_file_path, data_file_path):

        rospy.init_node("strategy")

        # TODO: Update data.json with proper values

        rospy.loginfo("SCEN: " + scenario_file_path)
        with open(scenario_file_path) as scenario_file:
            scenario = yaml.load(scenario_file)

        with open(data_file_path) as data_file:
            data = json.load(data_file)

        self.base_x = data["base"]["x"]
        self.base_y = data["base"]["y"]
        self.base_alpha = data["base"]["alpha"]

        self.shelf_positions = data["shelf_positions"]
        self.trophy_positions = data["trophy_positions"]
        self.trophy_heights = data["trophy_heights"]

        self.shelf_width = data["env"]["shelf_width"]
        self.max_deploy = data["env"]["max_deploy"]
        self.max_neighborhood_score = data["env"]["max_neighborhood_score"]
        self.arm_height_drop = data["env"]["arm_height_drop"]

        # Current Mode of Robot
        # 0 = Idle
        # 1 = Robot Moving
        # 2 = Arm Moving
        # 3 = Robot Gripping
        self.mode = 0

        # Current Phase of Strategy
        # -1 = Loading
        # 0 = Get Trophy
        # 1 = Return Trophy
        # 2 = Finished
        self.phase = -1

        # List of all Trophies, Constantly Updated
        self.trophy_list = []
        for s in scenario["scenario"]:
            shelf_id = s["id"]
            for trophy in s["trophies"]:
                t = Trophy(
                    trophy_id="{}{}0".format(shelf_id, trophy),
                    x=self.shelf_positions[str(shelf_id)]["x"],
                    y=self.shelf_positions[str(shelf_id)]["y"],
                    z=self.trophy_heights[str(trophy)],
                    shelf=shelf_id,
                    level=trophy,
                    w=0
                )
                self.trophy_list.append(t)
        self.trophy_map = None
        self.neighbor_score_mask = data["neighbor_score_mask"]
        self.update_trophy_map()
        self.trophy_goal = None

        # Send request to publish the travel cost to different shelves
        self.pub_travel = rospy.Publisher('/base_cntrl/in_cmd', String, queue_size=3)
        # Send request to drive to position
        self.pub_base = rospy.Publisher('/base_cntrl/go_to_pose', Pose, queue_size=3)
        # Send request for arm to move to certain height
        self.pub_arm = rospy.Publisher('/arm_cmd', Int16, queue_size=3)
        # Send request to grip or drop trophy
        self.pub_gripper = rospy.Publisher('/gripper_cmd', String, queue_size=3)

        # Receive list of travel costs to different shelves
        self.sub_travel = rospy.Subscriber("/base_cntrl/cost_list", Float32MultiArray, self.score)
        # Receive information that base is in position
        self.sub_base = rospy.Subscriber("/base_cntrl/out_result", String, self.base_in_position)
        # Receive information that gripper completed task
        self.sub_gripper = rospy.Subscriber("/manipulator/gripper_result", Bool, self.gripper_in_position)
        # Receive updates about the trophies from perception
        self.sub_trophy = rospy.Subscriber("/trophy_update", String, self.trophy_update)

        rospy.loginfo("Waiting for loading procedure to finish")
        rate = rospy.Rate(2.0)
        while not rospy.get_param('target_spawning_complete', False):
            rate.sleep()
        rospy.loginfo("Loading completed")

        self.retract_arm()

    def update_trophy_map(self):
        self.trophy_map = np.zeros((4, 8))
        for trophy in self.trophy_list:
            self.trophy_map[int(trophy.level), int(trophy.shelf)-1] = self.trophy_map[int(trophy.level), int(trophy.shelf)-1] + 1

    """
    def score_one(self):
        max_val = (0, None)
        for trophy in self.trophy_list:
            deploy_time = self.calculate_deploy_time(trophy)
            n_density = self.calculate_n_density(trophy)
            # info_gain = self.calculate_info_gain
            score = 1.0 * (1 - (deploy_time / self.max_deploy)) \
                + 0 * (n_density / self.max_neighborhood_score)
            if score > max_val[0]:
                max_val = (score, trophy)
        return max_val[1]

    def score_two(self):
        max_val = (0, None)
        for trophy in self.trophy_list:
            deploy_time = self.calculate_deploy_time(trophy)
            n_density = self.calculate_n_density(trophy)
            difficulty = self.calculate_difficulty(trophy)
            score = (1.0 * (1 - (deploy_time / self.max_deploy))
                     + 0 * (n_density / self.max_neighborhood_score)) * (1 - difficulty)
            if score > max_val[0]:
                max_val = (score, trophy)
        return max_val[1]
    """

    def travel_times(self):
        self.pub_travel.publish(String("get_cost_of_travel"))

    def score(self, msg):
        travel_times = msg.data
        max_val = (0, None)
        for trophy in self.trophy_list:
            deploy_time = travel_times[trophy.shelf]
            n_density = self.calculate_n_density(trophy)
            difficulty = self.calculate_difficulty(trophy)
            score = (1.0 * (1 - (deploy_time / self.max_deploy))
                     + 0 * (n_density / self.max_neighborhood_score)) * (1 - difficulty)
            if score > max_val[0]:
                max_val = (score, trophy)
        self.trophy_goal = max_val[1]
        self.move_base()

    def calculate_deploy_time(self, trophy):
        # TODO: Either load deploy time from file or request from path planning
        pass

    def calculate_n_density(self, trophy):
        if trophy.shelf == 1:
            map_values = self.trophy_map[:, :2]
            mask_result = np.multiply(map_values, np.array(self.neighbor_score_mask[str(trophy.level)])[:, 1:])
            density = np.sum(mask_result)
            pass
        elif trophy.shelf == 8:
            map_values = self.trophy_map[:, 6:]
            mask_result = np.multiply(map_values, np.array(self.neighbor_score_mask[str(trophy.level)])[:, :2])
            density = np.sum(mask_result)
        else:
            map_values = self.trophy_map[:, (trophy.shelf - 2): (trophy.shelf + 1)]
            mask_result = np.multiply(map_values, np.array(self.neighbor_score_mask[str(trophy.level)]))
            density = np.sum(mask_result)
        return density

    def calculate_info_gain(self, trophy):
        # Relict, not expected to be implemented
        pass

    def calculate_difficulty(self, trophy):
        difficulty = (trophy.w / (self.shelf_width / 2)) ** 8
        return difficulty

    """
    def move_base(self, x, y, alpha):
        quaternion = euler_to_quaternion(0, 0, alpha)
        pose = (Pose(Point(x, y, 0.0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])))
        self.mode = 1
        self.pub_base.publish(pose)

    def move_base_to_goal(self):
        x = self.shelf_positions[str(self.trophy_goal.shelf)]["x"]
        y = self.shelf_positions[str(self.trophy_goal.shelf)]["y"]
        alpha = self.shelf_positions[str(self.trophy_goal.shelf)]["alpha"]
        self.move_base(x, y, alpha)
    """

    def retract_arm(self):
        self.pub_gripper.publish(String("fold"))

    def move_base(self):
        self.pub_travel.publish(String("shelf{}".format(self.trophy_goal.shelf)))

    def return_base(self):
        # self.move_base(self.base_x, self.base_y, self.base_alpha)
        self.pub_travel.publish(String("bin"))

    def trophy_update(self, msg):
        # TODO: Create list of Trophy objects from input and compare it with current list
        # TODO: If the goal trophy is at a significantly different place than currently thought: update
        # TODO: If the goal trophy does not exist: abort
        trophy_list = json.loads(msg.data)
        if self.phase == -1:
            pass
        elif self.phase == 0:
            # Compare with old trophy list
            # If the current goal trophy is no longer on the list: choose new trophy
            # If the coordinates of the trophy have been updated beyond a certain degree: send new goal to arm/base
            pass
        elif self.phase == 1:
            # Simply update the trophy list
            pass

    def base_in_position(self, msg):
        message = msg.data
        if message == "OK MOVE":
            if self.phase == -1:
                pass
            elif self.phase == 0:
                self.pub_arm.publish(Int16(int(self.trophy_goal.level)))
                self.pub_gripper.publish(String("grip"))
            elif self.phase == 1:
                self.pub_gripper.publish("deposit")

    def gripper_in_position(self, msg):
        result = msg.data
        if self.phase == -1:
            self.phase = 0
            self.travel_times()
        elif self.phase == 0:
            if result:
                self.phase = 1
                self.return_base()
            else:
                self.travel_times()
        elif self.phase == 1:
            self.phase = 0
            self.travel_times()


if __name__ == '__main__':
    scenario_file_arg = sys.argv[1]
    data_file_arg = sys.argv[2]
    try:
        Strategy(scenario_file_arg, data_file_arg)
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Strategy terminating.")
