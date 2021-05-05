#!/usr/bin/env python
# SA and FT, 4/5/21
# Automate scouting process

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import String, Int16
import enum
import numpy as np
import sys

debug_output = True
# Wait for tf to catch up.
# TODO Is this long enough/too long?
wait_time = 0.5

# Allows skipping of angles which don't have any shelves.
#                  30  60  90  120 150 210 240 270 300 330 360
rotation_angles = [30, 30, 30, 30, 30, 60, 30, 30, 30, 30, 30]  # Degrees


class Phases(enum.Enum):
    INITIAL = 1
    SCOUTING = 2
    COMPLETE = 3


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(qx, qy, qz, qw)


def wait_for_start(msg_string):
    if (msg_string.data == "scout"):
        # Drive to scout pose
        scout_pose = Pose(Point(-0.5, -2.0, 0.0), euler_to_quaternion(0, 0, 0))
        move_request_pub.publish(scout_pose)

        if debug_output:
            print("Scouting: starting...")


def move_confirmed(msg_string):
    global phase
    global rotation_index
    global total_rotation

    if (phase == Phases.INITIAL):
        if (msg_string.data == "OK MOVE"):
            phase = Phases.SCOUTING
            # Request first image
            image_request_pub.publish("Image_request")
        else:
            strat_notifier.publish("BAD scouting")
    elif (phase == Phases.SCOUTING) and (msg_string.data == "OK ROTATE"):
        # A rotation has been completed.
        total_rotation += rotation_angles[rotation_index]

        if (debug_output):
            print("Scouting: one {}d rotation complete".format(
                rotation_angles[rotation_index]))
            print("Scouting: turned through total rotation: {}d".format(
                total_rotation))  # Debug

        rotation_index += 1

        rospy.sleep(wait_time)

        if (rotation_index == len(rotation_angles)):
            if (debug_output):
                print("Scouting: rotation complete")
            phase = Phases.COMPLETE
            # Publish that scouting is complete.
            strat_notifier.publish("completed scouting")
            # Shutting the node down [else there is erronous behaviour later on,
            # when subsequent messages are sent on the subscribed topics].
            # TODO Do we need to shut the publishers and subscribers down cleanly?
            sys.exit(0)
        else:
            # Request next image.
            if (debug_output):
                print("Scouting: requesting image")
            image_request_pub.publish("Image_request")


def image_taken(msg_string):
    global rotation_index

    if (msg_string.data == "Trophy_estimates_obtained"):
        if (debug_output):
            print("Scouting: image confirmation received")

        # Request the next rotation (see rotation_angles)
        if (debug_output):
            print("################# Scouting:: rotation_index: {}".format(
                rotation_index))
        turn_request_pub.publish(rotation_angles[rotation_index])


phase = Phases.INITIAL
# Index of the *next* rotation to be performed.
rotation_index = 0
total_rotation = 0

# Receive a request to begin scouting process.
rospy.init_node("scouting")

# Topic for beginning scouting operation.
scout_start = rospy.Subscriber(
    "/base_cntrl/in_cmd", String, wait_for_start, queue_size=1)

move_request_pub = rospy.Publisher("/base_cntrl/go_to_pose/", Pose)
turn_request_pub = rospy.Publisher("/base_cntrl/rotate_deg", Int16)
movement_confirmation = rospy.Subscriber(
    "/base_cntrl/out_result", String, move_confirmed, queue_size=1)
# For communicating end of scouting to Strategy.
strat_notifier = rospy.Publisher("/base_cntrl/out_result", String)
image_request_pub = rospy.Publisher(
    "/trophy_image_robot_request_response", String)
image_confirmation = rospy.Subscriber(
    "/trophy_image_robot_request_response", String, image_taken, queue_size=1)

# TODO Does it slow things down to put a 'sleep' here?
# rospy.sleep(1.0)

if (debug_output):
    print("Scouting: waiting to start...")

while not rospy.is_shutdown():
    rospy.spin()
