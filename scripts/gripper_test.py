#!/usr/bin/env python
import rospy
from farscope_group_project.farscope_robot_utils import ArmMover, GripperController, BaseDriver
from std_msgs.msg import Bool

# Create ROS node & topic publisher
rospy.init_node("test_pickup")
pub = rospy.Publisher("object_grip", Bool)
rate = rospy.Rate(0.5)  # Set rate of publishing in hz - 0.5 = 1 per 2 seconds

# Intialise arm, base and gripper controller objects
arm_mover = ArmMover()
base_driver = BaseDriver()
gripper_controller = GripperController()

# Starting position of robot arm
# Shab says arm must be folded up for travel?


# Hardcoded shelf heights obtained by trial and error within simulator
# Shoulder + Elbow is used by Arthur as he uses the base of robot to move the arm in and out of the shelf
# Do we want to follow this?

# Shoulder + Elbow:
# Bottom (0) = shoulder_lift_cmd_in=0.15, elbow_cmd_in=1.0
# 2nd (1) = shoulder_lift_cmd_in=-0.25, elbow_cmd_in=1.0
# 3rd (1) = shoulder_lift_cmd_in=-0.6, elbow_cmd_in=1.0
# Top (1) = shoulder_lift_cmd_in=-1.0, elbow_cmd_in=1.0
# In testing elbow joint remained fixed for different heights
shoulder_heights = [0.15, -0.25, -0.6, -1.0]

# Publish 0 to indicate no object is grasped at start of script
pub.publish(0)

target_shelf = 0    # Dummy value

# Loop while ros script is running
while not rospy.is_shutdown():

    # Wait to be sent the target shelf
    #target_shelf = rospy.wait_for_message(topic, shelf_index)
    # THIS NEEDS WORK

    # Open gripper completely
    gripper_controller.open()

    # Move arm to intended height
    arm_mover.move(
    shoulder_lift_cmd_in=shoulder_heights[target_shelf], elbow_cmd_in=1.0)

    # Move robot into shelf to grab object
    # THIS NEEDS WORK
    # Current base movements are taken form Arthurs example
    base_driver.move(0.3, -0.0)
    #base_driver.move(0.3, -0.1)
    base_driver.move(0.3, -0.00)
    #base_driver.move(0.3, -0.05)

    # Grip object
    gripper_controller.close()

    # Publish that object is grasped
    # Publish a bool of 1 to show object is gripped
    pub.publish(1)

    # Slight lift off the shelf
    arm_mover.move(shoulder_lift_cmd_in=(shoulder_heights[target_shelf]-0.05), elbow_cmd_in=1.0)

    # Back out of shelf
    base_driver.move(-0.3, 0, 0, 2)

    # Pass control back to the navigation team to take us home
    # Do we have to fold up the arm to move? Can we fold up the arm while having a trophy in hand?

    # Wait for message saying we're home before dropping object and folding up arm
    #deposit = rospy.wait_for_message(navigation_topic, deposit)

    # Unfold arm

    # Open gripper to drop object in target area
    gripper_controller.open()
    pub.publish(0)  # Publish 0 value to show no object is gripped

    # Fold up arm

    # Demo loop
    target_shelf = target_shelf + 1