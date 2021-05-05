#!/usr/bin/env python
import rospy
from farscope_project_entry.farscope_robot_utils import ArmMover
from std_msgs.msg import String

class arm_fold():
    def __init__(self):
        # Start arm_fold node
        rospy.init_node("arm_fold")

        # Arm Control Object
        self.arm_mover = ArmMover()
        
        # Subscribe to /arm_cmd topic
        self.arm_cmd = rospy.Subscriber("/arm_cmd", String, self.move)

    def move(self,msg):
        command = msg.data

        if command == "fold":
            self.arm_mover.move(shoulder_lift_cmd_in = -2.40, elbow_cmd_in=2.34, wrist_2_cmd = 3.14)



# Main function of script
if __name__ == '__main__':
    try:
        arm_fold()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("ARM FOLD NODE TERMINATING.")
