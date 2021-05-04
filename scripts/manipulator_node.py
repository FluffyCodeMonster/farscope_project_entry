#!/usr/bin/env python
import rospy
from farscope_project_entry.farscope_robot_utils import ArmMover, GripperController, BaseDriver
from std_msgs.msg import Bool, String, Int16, Float32

# Create class for the manipulator


class Manipulator:
    def __init__(self):
        # Initialise node
        rospy.init_node("manipulator")

        # Create publisher topics
        # Arm status published as a string topic
        #   "STARTING GRIP"
        #   "ARM @ SHELF"
        #   "MOVING INTO SHELF"
        #   "OBJECT GRIPPED"
        #   "MOVING OUT SHELF"
        #   "READY TO MOVE"
        #   "ITEM DEPOSITED"
        #   "UNFOLDING"
        #   "ARM FOLDED"
        #   "READY TO MOVE"
        self.arm_status = rospy.Publisher(
            "/manipulator/arm_status", String, queue_size=3)
        self.gripper_result = rospy.Publisher(
            "/manipulator/gripper_result", Bool, queue_size=3)

        # Create controller objects for base, gripper and base
        self.arm_mover = ArmMover()
        self.base_driver = BaseDriver()
        self.gripper_controller = GripperController()

        # Hardcoded shelf heights
        self.shoulder_heights = [0.15, -0.25, -0.6, -1.0]

        # Subscribe to topics from the strat team
        # Callbacks on messages recieved
        self.shelf_sub = rospy.Subscriber(
            "/arm_cmd", Int16, self.shelf_selection)
        self.gripper_cmd = rospy.Subscriber(
            "/gripper_cmd", String, self.selection)
        self.adjust_sub = rospy.Subscriber(
            "/perception_adjust", Float32, self.adjust_and_grip)

        # Log info
        self.arm_log("Initialising Manipulator node")

        self.taget_shelf = 0
        self.adjust_float = 0

    # Selection function takes input from strategy and choose a routine to run
    def selection(self, msg):
        command = msg.data

        rospy.loginfo("Command = %s", command)   # Debug

        if command == "grip":
            self.arm_to_shelf()

        elif command == "deposit":
            self.deposit()

        elif command == "fold":
            self.fold_arm()

        else:
            self.arm_log("READY")

    # Selection function takes input from strategy and sets the target shelf for the pickup routine
    def shelf_selection(self, msg):
        self.taget_shelf = msg.data

        rospy.loginfo("Target Shelf = " + str(self.taget_shelf))   # Debug

    # Function runs a apickup routine @ a certain shelf height

    def arm_to_shelf(self):

        # Move base back to avoid collision
        self.base_driver.move(-0.35, 0, 0, 2)

        # Unfold wrist
        self.arm_mover.move(wrist_2_cmd=1.6)

        # Publish status as we go "/arm_status"
        self.arm_log("STARTING GRIP")

        # Move arm to intended height
        self.arm_mover.move(
            shoulder_lift_cmd_in=self.shoulder_heights[self.taget_shelf], elbow_cmd_in=1.0, wrist_2_cmd=1.6)

        self.arm_log("ARM @ SHELF")

    # Function moves arm into a position for transit
    # Camera currently facing forwards

    def fold_arm(self):
        self.arm_log("FOLDING ARM")
        self.arm_mover.move(shoulder_lift_cmd_in=-2.40,
                            elbow_cmd_in=2.4, wrist_2_cmd=3.14)
        self.arm_log("ARM FOLDED")

    # Function to unfold the arm for deposit
    def unfold_arm(self):
        self.arm_log("UNFOLDING ARM")
        self.arm_mover.move(shoulder_lift_cmd_in=0,
                            elbow_cmd_in=0, wrist_2_cmd=1.6)
        self.arm_log("ARM UNFOLDED")

    # Function for the deposit routine
    # Currently a dummy routine
    def deposit(self):
        self.unfold_arm()
        self.gripper_controller.open()
        self.arm_log("ITEM DEPOSITED")
        # Send message to strategy team to indicate deposit
        self.gripper_result.publish(False)
        self.fold_arm()

    # Function logs the string input to rosout & /arm_status
    def arm_log(self, message):
        rospy.loginfo(message)
        self.arm_status.publish(message)

    # Function to continue gripping task

    def adjust_and_grip(self, msg):
        # Adjust in the x plane before moving into shelf
        data = msg.data
        self.base_driver(0, data, 0, 2)
        self.arm_log("BASE ADJUSTED")

        # Move robot into shelf to grab object
        self.arm_log("MOVING INTO SHELF")
        # THIS NEEDS WORK
        # Current base movements are taken form Arthurs example
        self.base_driver.move(0.2, -0.0)
        self.base_driver.move(0.1, -0.00)
        self.arm_log("GRIPPING")

        # Grip object
        self.gripper_controller.close()

        # Publish that object is grasped
        self.arm_log("OBJECT GRIPPED")

        # Slight lift off the shelf
        self.arm_mover.move(shoulder_lift_cmd_in=(
            self.shoulder_heights[self.taget_shelf]-0.05), elbow_cmd_in=1.0, wrist_2_cmd=1.6)

        self.arm_log("MOVING OUT SHELF")

        # Back out of shelf
        self.base_driver.move(-0.3, 0, 0, 2)

        self.fold_arm()

        # Send message to strategy team to indicate success
        self.gripper_result.publish(True)


# Main function of script
if __name__ == '__main__':
    try:
        Manipulator()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("MANIPULATOR NODE TERMINATING.")
