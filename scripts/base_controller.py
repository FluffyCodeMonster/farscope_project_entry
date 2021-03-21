#!/usr/bin/env python
import sys, rospy
from std_msgs.msg import String
from farscope_group_project.farscope_robot_utils import BaseDriver

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
#from tf.transformations import quaternion_from_euler
#from tf_conversions.transformations import quaternion_from_euler
import tf_conversions as tf_conv
#tf_conversions.transformations.quaternion_from_euler(

from visualization_msgs.msg import Marker
from math import radians, pi

class BaseController:
    def __init__(self):
        # This will be our node name
        rospy.init_node("base_controller")

        # We will subscribe to a String command topic
        self.base_sub = rospy.Subscriber("/base_cntrl/in_cmd", String, self.on_command)

        # We will publish a String feedback topic
        self.base_pub = rospy.Publisher("/base_cntrl/out_result", String, queue_size=3)
        
        # We will publish a String feedback topic
        self.goal_setter = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=3)

        # Base driver object to allow for a simple way to request movement from the actual robot
        self.base_driver = BaseDriver()

        # We will publishing to cmd_vel directly when shutting down as a crude way to stop the robot in its tracks if it is moving.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)
        
        # Subscribe to the move_base action server. This lets us to directly interact with move_base- to ask for paths and set goals.
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Clean up on shutdown        
        rospy.on_shutdown(self.shutdown)
        
        # Now that all the admin stuff has been done, let's do some actual business logic.
        # Let's prepare goals.
        self.prepare_goals()

        # wait for parameter that says spawning is finished
        rospy.loginfo("Waiting for spawning to finish")
        rate = rospy.Rate(2.0)
        while not rospy.get_param('target_spawning_complete',False):
            rate.sleep()
        rospy.loginfo("Spawning completion detected - time to get going!")

    # This will prepare goals and the end angles (think shelf #1, #2, etc)
    def prepare_goals(self):
        # Create a list to hold the target quaternions (orientations)
#        quaternions = list()
#        
#        # First define all our shelf orientations as Euler angles
#        euler_angles = [pi/2, pi, 3*pi/2, 0.]
#        
#        # Then convert the angles to quaternions
#        for angle in euler_angles:
#            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
#            q = Quaternion(*q_angle)
#            quaternions.append(q)
            
        # Now let's create a few shelves (their poses that is)
        self.shelves = list()
        #self.shelves.append(Pose(Point(2.0, -3.0, 0.0), quaternions[0]))
        # Position of shelf #3 : location: {x: 2.0, y: -3.0, ang: -1.5706}
        quat = tf_conv.transformations.quaternion_from_euler(0, 0, -1.5706, axes='sxyz')
        self.shelves.append(Pose(Point(0.0, -2.5, 0.0), Quaternion(quat[0], quat[1], quat[2], quat[3])))
        
        # Goal ID
        self.goal_id = 0

    # This is how we'll react on the commands received
    def on_command(self, cmd):
        if cmd.data == "fwd":
            self.base_driver.move(1.0)
            self.base_pub.publish("OK FWD")
        elif cmd.data == "back":
            self.base_driver.move(-1.0)
            self.base_pub.publish("OK BACK")
        elif cmd.data == "right":
            self.base_driver.move(0, -1.0)
            self.base_pub.publish("OK RIGHT")
        elif cmd.data == "left":
            self.base_driver.move(0, 1.0)
            self.base_pub.publish("OK LEFT")
        elif cmd.data == "spin":
            self.base_driver.move(0, 0, 0.1, 105)
            self.base_pub.publish("OK SPIN")
        elif cmd.data == "shelf3":
            # Intialize the goal
            goal = MoveBaseGoal()
            
            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'odom'
            
            # Set the goal ID
            self.goal_id += 1
            goal.target_pose.header.seq = self.goal_id
            
            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()
        
            rospy.loginfo("AAAAAAA")
            
            print(self.shelves[0])
            # Set the goal pose to the shelf
            goal.target_pose.pose = self.shelves[0]
            
            # Start the robot moving toward the goal
            self.move(goal)
            self.base_pub.publish("OK SHELF3")
    
    def move2(self, goal):
        mbag = MoveBaseActionGoal()
        
        # First Header
        mbag.header.frame_id = '' # Use the map frame to define goal poses
        mbag.header.seq = self.goal_id
        mbag.header.stamp = rospy.Time.now()
        
        # Now the Goal ID
        mbag.goal_id.stamp = rospy.Time.now()
        mbag.goal_id.id = "g_" + str(self.goal_id)
        
        # Now the goal, which we already have as an argument
        mbag.goal = goal
        
        #PoseStamped(Pose(Point(2.0, -3.0, 0.0), quaternion_from_euler(0, 0, -1.5706, axes='sxyz')))
        self.goal_setter.publish(mbag)
    
    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
    
    def shutdown(self):
        rospy.loginfo("Stopping base controller...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
if __name__ == '__main__':
    try:
        BaseController()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Base Controller terminating.")
