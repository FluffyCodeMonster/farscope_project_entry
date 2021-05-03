#!/usr/bin/env python

from std_msgs.msg import String, Float32, Int16, Float32MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
import rospy
import json
import sys
import numpy as np
import copy


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
    def __init__(self, data_file_path):

        rospy.init_node("strategy")

        # TODO: Update data.json with proper values

        with open(data_file_path) as data_file:
            data = json.load(data_file)

        self.base_x = data["base"]["x"]
        self.base_y = data["base"]["y"]
        self.base_alpha = data["base"]["alpha"]

        self.shelf_positions = data["shelf_positions"]
        self.trophy_positions = data["trophy_positions"]
        self.trophy_heights = data["trophy_heights"]

        self.shelf_width = data["env"]["shelf_width"]
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
        self.trophy_map = None
        self.neighbor_score_mask = data["neighbor_score_mask"]
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
            self.trophy_map[int(trophy.level), int(trophy.shelf)-1] = self.trophy_map[int(trophy.level),
                                                                                      int(trophy.shelf)-1] + 1

    def retract_arm(self):
        self.pub_gripper.publish(String("fold"))

    def go_scouting(self):
        self.pub_base.publish(String("scout"))

    def travel_times(self):
        self.pub_travel.publish(String("get_cost_of_travel"))

    def score(self, msg):
        travel_times = msg.data
        max_val = (0, None)
        for trophy in self.trophy_list:
            deploy_time = travel_times[trophy.shelf]
            n_density = self.calculate_n_density(trophy)
            difficulty = self.calculate_difficulty(trophy)
            score = (1.0 * (1 - (deploy_time / max(travel_times)))
                     + 0 * (n_density / self.max_neighborhood_score)) * (1 - difficulty)
            if score > max_val[0]:
                max_val = (score, trophy)
        self.trophy_goal = max_val[1]
        self.move_base()

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

    def calculate_difficulty(self, trophy):
        difficulty = (trophy.w / (self.shelf_width / 2)) ** 8
        return difficulty

    def move_base(self):
        self.pub_travel.publish(String("shelf{}".format(self.trophy_goal.shelf)))

    def return_base(self):
        # self.move_base(self.base_x, self.base_y, self.base_alpha)
        self.pub_travel.publish(String("bin"))

    def trophy_update(self, msg):
        trophy_list = json.loads(msg.data)
        old_trophy_list = copy.deepcopy(self.trophy_list)
        for trophy in trophy_list.values():
            shelf = trophy[0]
            level = trophy[1]
            pos = self.shelf_width - trophy[2]
            coord = trophy[3]
            new = True
            for i, t in enumerate(old_trophy_list):
                if t.shelf == shelf & t.level == level & ((t.w - pos) < 0.1):
                    new_trophy = Trophy(
                        trophy_id=t.trophy_id,
                        x=coord[0],
                        y=coord[1],
                        z=coord[2],
                        shelf=shelf,
                        level=level,
                        w=pos
                    )
                    self.trophy_list[i] = new_trophy
                    new = False
            if new:
                trophy_id = "{}{}{}".format(level, shelf, self.trophy_map[level, shelf])
                new_trophy = Trophy(
                    trophy_id=trophy_id,
                    x=coord[0],
                    y=coord[1],
                    z=coord[2],
                    shelf=shelf,
                    level=level,
                    w=pos
                )
                self.trophy_list.append(new_trophy)
        self.update_trophy_map()

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
        elif message == "completed scouting":
            self.phase = 0
            self.travel_times()

    def gripper_in_position(self, msg):
        result = msg.data
        if self.phase == -1:
            self.go_scouting()
        elif self.phase == 0:
            if result:
                self.phase = 1
                self.return_base()
            else:
                self.travel_times()
        elif self.phase == 1:
            self.phase = 0
            self.travel_times()

    def gripper_adjustment(self, msg):
        for trophy in self.trophy_list:
            if trophy.trophy_id == self.trophy_goal.trophy_id:
                pass
                # publish: trophy.w


if __name__ == '__main__':
    data_file_arg = sys.argv[1]
    try:
        Strategy(data_file_arg)
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Strategy terminating.")
