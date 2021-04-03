#!/usr/bin/env python
import rospy
from farscope_group_project.farscope_robot_utils import ArmMover, GripperController, BaseDriver
from std_msgs.msg import Bool, String, Int16

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
        #   
        self.arm_status = rospy.Publisher("/arm_status", String, queue_size=3)
        self.gripper_result = rospy.Publisher("/gripper_result", Bool, queue_size=3)

        # Create controller objects for base, gripper and base
        self.arm_mover = ArmMover()
        self.base_driver = BaseDriver()
        self.gripper_controller = GripperController()

        # Hardcoded shelf heights
        self.shoulder_heights = [0.15, -0.25, -0.6, -1.0]

        # Publish to topics to indicate status
        # Publishing to the topics at the start of the script may not publish correctly
        self.gripper_result.publish("null")

        # Subscribe to topics from the strat team
        # Callback on gripper_cmd
        self.shelf_sub = rospy.Subscriber("/arm_cmd", Int16)
        self.gripper_cmd = rospy.Subscriber("/gripper_cmd", String, self.selection)
        
        # Log info
        self.arm_log("Initialising Manipulator node")
    

    # Selection function takes input from strategy and choose a routine to run
    def selection(self, msg):
        command = msg.data

        rospy.loginfo("Command = %s",command)   # Debug

        if command == "grip":
            self.pickup_routine()

        elif command == "deposit":
            self.deposit()

        else:
            self.arm_log("READY")


    # Function runs a apickup routine @ a certain shelf height
    def pickup_routine(self):

        # Publish status as we go "/arm_status"
        self.arm_log("STARTING GRIP")

        # Move arm to intended height
        self.arm_mover.move(shoulder_lift_cmd_in = self.shoulder_heights[self.shelf_sub], elbow_cmd_in=1.0)

        self.arm_log("ARM @ SHELF")
        self.arm_log("MOVING INTO SHELF")

        # Move robot into shelf to grab object
        # THIS NEEDS WORK
        # Current base movements are taken form Arthurs example
        self.base_driver.move(0.3, -0.0)
        #base_driver.move(0.3, -0.1)
        self.base_driver.move(0.3, -0.00)
        #base_driver.move(0.3, -0.05)

        self.arm_log("GRIPPING")

        # Grip object
        self.gripper_controller.close()

        # Publish that object is grasped
        self.arm_log("OBJECT GRIPPED")

        # Slight lift off the shelf
        self.arm_mover.move(shoulder_lift_cmd_in=(self.shoulder_heights[self.shelf_sub]-0.05), elbow_cmd_in=1.0)

        self.arm_log("MOVING OUT SHELF")

        # Back out of shelf
        self.base_driver.move(-0.3, 0, 0, 2)

        self.arm_log("FOLDING ARM")
        self.fold_arm()

        # Send message to gripper team to indicate success
        self.gripper_result.publish("success")


    # Function moves arm into a position for transit
    # Hardcoded to point straight upwards currently
    # NEEDS WORK
    def fold_arm(self):
        self.arm_log("FOLDING ARM")
        #self.arm_mover.move(shoulder_lift_cmd_in = 0, elbow_cmd_in=1.0)
        self.arm_log("ARM FOLDED")

    
    # Function for the deposit routine
    # Currently a dummy routine
    def deposit(self):
        self.arm_log("ITEM DEPOSITED")
        self.fold_arm()

    # Function logs the string input to rosout & /arm_status
    def arm_log(self, message):
        rospy.loginfo(message)
        self.arm_status.publish(message)



# Main function of script
if __name__ == '__main__':
    try:
        Manipulator()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("MANIPULATOR NODE TERMINATING.")
