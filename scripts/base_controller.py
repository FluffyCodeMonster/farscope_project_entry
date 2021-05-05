#!/usr/bin/env python
import sys, rospy
from std_msgs.msg import String, Int16, Float32, Float32MultiArray, MultiArrayDimension
#from farscope_group_project.farscope_robot_utils import BaseDriver
from farscope_project_entry.farscope_robot_utils import BaseDriver

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
#from tf.transformations import quaternion_from_euler
#from tf_conversions.transformations import quaternion_from_euler
#import tf_conversions as tf_conv
#tf_conversions.transformations.quaternion_from_euler(

from math import radians, pi, sqrt
import numpy as np

# the robt base will position itself 1m from each shelf
off_set = 1.0

# Angles need to be added PI because they're the angles of where shelves are facing, but our robot needs to face the opposite way- towards the faces of the shelves.
# Labels              x                   y               angle
shelf_pose =   [[2.0 - off_set,     0.0,                -pi/2 + pi],
                [2.0 - off_set,     -1.5,               -pi/2 + pi],
                [2.0 - off_set,     -3.0,               -pi/2 + pi],
                [1.0,               -4.0 + off_set,      pi   + pi],
                [-0.5,              -4.0 + off_set,      pi   + pi],
                [-2.0,              -4.0 + off_set,      pi   + pi],
                [-3.0 + off_set,    -3.0,                pi/2 + pi],
                [-2.0,              -1.0 - off_set,      0.0  + pi],
                [1.0,               1.5 - off_set,       0.0  + pi]]

class BaseController:
    # Number of seconds required to rotate 1 degree at speed of 0.1 when using base_driver.move.
    # E.g. to rotate 15 degrees to the right we'll need: self.base_driver.move(0, 0, 0.1, 15 * ROT_1_DEG_TIME)
    ROT_1_DEG_TIME = 0.3
    
    def __init__(self):
        # This will be our node name
        rospy.init_node("base_controller")

        # We will subscribe to a String command topic
        self.base_sub = rospy.Subscriber("/base_cntrl/in_cmd", String, self.on_command)
        
        # We will subscribe to an Int16 command topic to rotate cw or ccw by the passed number of degrees.
        self.rotate_sub = rospy.Subscriber("/base_cntrl/rotate_deg", Int16, self.on_rotate)
        
        # We will subscribe to an Int16 command topic to move right by the passed amount of meters.
        self.go_right_sub = rospy.Subscriber("/base_cntrl/go_right", Float32, self.on_right)
        
        # We will subscribe to an Int16 command topic to move left by the passed amount of meters.
        self.go_left_sub = rospy.Subscriber("/base_cntrl/go_left", Float32, self.on_left)
        
        # We will subscribe to an Int16 command topic to move forward by the passed amount of meters.
        self.go_fwd_sub = rospy.Subscriber("/base_cntrl/go_fwd", Float32, self.on_fwd)
        
        # We will subscribe to an Int16 command topic to move backwards by the passed amount of meters.
        self.go_back_sub = rospy.Subscriber("/base_cntrl/go_back", Float32, self.on_back)
        
        # We will subscribe to an Pose command topic to move backwards by the passed amount of meters.
        self.move_to_pose_sub = rospy.Subscriber("/base_cntrl/go_to_pose", Pose, self.move_to_pose)
        
        # We will subscribe to an Pose command topic to scout to a given pose.
        self.scout_to_pose_sub = rospy.Subscriber("/base_cntrl/scout_to_pose", Pose, self.scout_to_pose)

        # We will subscribe to a PoseWithCovarianceStamped command topic to have access to the current position of robot.
        self.amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.robot_pose)

        # We will subscribe to an Int16 command topic to move backwards by the passed amount of meters.
        self.path_cost_qry_sub = rospy.Subscriber("/base_cntrl/path_cost_qry", Pose, self.publish_path_cost)
        
        # Getting the service for path planning
        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
        # Service for clearing unknown space since we're not going to have dynamic obstacles
        #self.space_clear = rospy.ServiceProxy('/move_base/clear_unknown_space', Empty)
        
        # Service for clearing costmaps
        self.costmap_clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # We will publish a Float32 value as a response to the path cost query
        self.path_cost_pub = rospy.Publisher("/base_cntrl/path_cost", Float32, queue_size=3)

        # We will publish a String feedback topic
        self.base_pub = rospy.Publisher("/base_cntrl/out_result", String, queue_size=3)

        # We will publish a String feedback topic
        self.goal_setter = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=3)

        # Base driver object to allow for a simple way to request movement from the actual robot
        self.base_driver = BaseDriver()

        # We will publishing to cmd_vel directly when shutting down as a crude way to stop the robot in its tracks if it is moving.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

        # Publish list of cost to travel to each shelf
        self.get_cost_list = rospy.Publisher("/base_cntrl/cost_list", Float32MultiArray, queue_size=3)

        # Subscribe to the move_base action server. This lets us to directly interact with move_base- to ask for paths and set goals.
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # We'll be keeping the current robot pose in this variable. We'll populate it with AMCL subscription callback.
        # but for now it will be at the initial point.
        self.latest_pose_stamped = PoseStamped()
        
        # First Header
        self.latest_pose_stamped.header.frame_id = ""
        self.latest_pose_stamped.header.seq = 0
        self.latest_pose_stamped.header.stamp = rospy.Time.now()
        
        # Now the actual pose
        self.latest_pose_stamped.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

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
        
        # init ros standard array message
        self.multiArray = Float32MultiArray()
        self.multiArray.data = list()
        allocate_array = MultiArrayDimension()
        allocate_array.label = "costs"
        allocate_array.size = 1
        allocate_array.stride = 1
        self.multiArray.layout.dim.append(allocate_array)

    # Conversion of Euler angle to a quaternion
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(qx, qy, qz, qw)

    # Multiplication of quaternions
    def multQ(self, Q1, Q2):
        x1 = Q1.x
        y1 = Q1.y
        z1 = Q1.z
        w1 = Q1.w
        
        x2 = Q2.x
        y2 = Q2.y
        z2 = Q2.z
        w2 = Q2.w
        
        # Some quaternion multiplication maths as per Wikipedia.
        # a = w
        # b = x
        # c = y
        # d = z
        return Quaternion(
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        )

    # Rotates a given Quaternion around Z axis by the given amount of radians- a.k.a multiplies the source quaternion with the rotation quaternion.
    def rotateQuaternionAroundZ(self, quat, rot):
        #quat_star = Quaternion(-quat.x, -quat.y, -quat.z, quat.w)
        quat_rot = self.euler_to_quaternion(0, 0, rot)
        
        #result = self.multQ(self.multQ(quat_rot, quat), quat_star)
        result = self.multQ(quat, quat_rot)
        return result

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

        #self.shelves.append(Pose(Point(2.0 - offset, -3.0, 0.0), self.euler_to_quaternion(0,0,-1.8846))) # Shelf 3
        for pos in shelf_pose:
            self.shelves.append(Pose(Point(pos[0], pos[1], 0.0), self.euler_to_quaternion(0,0,pos[2]))) # Shelf x
        
        # Goal ID
        self.goal_id = 0
        
        # Scouting position
        self.scout_pose = Pose(Point(-0.5, -2.0, 0.0), self.euler_to_quaternion(0,0,0))

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
        elif cmd.data == "shelf1":
            self.move_to_pose(self.shelves[0])
        elif cmd.data == "shelf2":
            self.move_to_pose(self.shelves[1])
        elif cmd.data == "shelf3":
            self.move_to_pose(self.shelves[2])
        elif cmd.data == "shelf4":
            self.move_to_pose(self.shelves[3])
        elif cmd.data == "shelf5":
            self.move_to_pose(self.shelves[4])
        elif cmd.data == "shelf6":
            self.move_to_pose(self.shelves[5])
        elif cmd.data == "shelf7":
            self.move_to_pose(self.shelves[6])
        elif cmd.data == "shelf8":
            self.move_to_pose(self.shelves[7])
        elif cmd.data == "bin":
            # Going to bin often ends up in getting stuck, so in that case we'll go back to the center and then to the bin again.
            if not self.move_to_pose(self.shelves[8]):
                self.move_to_pose(self.scout_pose, False)
                
                # calling move to bin again recursively here
                self.on_command("bin")
                
        elif cmd.data == "get_cost_of_travel":
            # publish list of path costs to all shelves
            self.get_cost_list.publish(self.calculate_cost_of_travel())
        #elif cmd.data == "scout":
        #    if (self.move_to_pose(self.scout_pose, False)):
        #        self.base_driver.move(0, 0, 0.1, 105)
        #        self.base_pub.publish("completed scouting")
        #    else:
        #        self.base_pub.publish("BAD scouting")
    
    # When user wants the robot to rotate, then this will be called with the number of degrees passed.
    # Positive number: rotating clock wise, negative: rotating counter clock wise
    def on_rotate(self, degs):
        sign = -1.0
        if degs.data < 0.0:
            sign = 1.0
        self.base_driver.move(0, 0, sign * 0.1, abs(degs.data) * self.ROT_1_DEG_TIME)
        self.base_pub.publish("OK ROTATE")
    
    # When user wants the robot to go right, then this will be called with the number of meters passed.
    def on_right(self, dist):
        self.base_driver.move(0, -1.0 * dist.data)
        self.base_pub.publish("OK RIGHT")
    
    # When user wants the robot to go left, then this will be called with the number of meters passed.
    def on_left(self, dist):
        self.base_driver.move(0, dist.data)
        self.base_pub.publish("OK LEFT")
    
    # When user wants the robot to go forward, then this will be called with the number of meters passed.
    def on_fwd(self, dist):
        self.base_driver.move(dist.data)
        self.base_pub.publish("OK FWD")
        
    # When user wants the robot to go forward, then this will be called with the number of meters passed.
    def on_back(self, dist):
        self.base_driver.move(-1.0 * dist.data)
        self.base_pub.publish("OK BACK")
    
    # Moves base to the given goal. This is simpler than "move" function as it doesn't track the completion.
    def move_to_goal(self, goal):
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
    
    # Creates an MoveBaseGoal object from a Pose and moves to it
    def move_to_pose(self, pose, report=True):
    
        # Because the point on the robot, which we're trying to get to the target pose, is offset by an angle,
        # we need to rotate the given pose by around pi/2 counter clock wise.
        pose = Pose(pose.position, self.rotateQuaternionAroundZ(pose.orientation, -pi/2))
        
        # Intialize the goal
        goal = MoveBaseGoal()
        
        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'odom'
        
        # Set the goal ID
        self.goal_id += 1
        goal.target_pose.header.seq = self.goal_id
        
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()

        #print(pose)

        # Set the goal pose to the shelf
        goal.target_pose.pose = pose
        
        # Start the robot moving toward the goal
        return self.move(goal, report)

    # Allows to scout to an arbitrary pose    
    def scout_to_pose(self, pose):
        if (self.move_to_pose(pose, False)):
            self.base_driver.move(0, 0, 0.1, 105)
            self.base_pub.publish("completed scouting")
        else:
            self.base_pub.publish("BAD scouting")
    
    # Moves the base to the passed goal, which includes a Pose
    # report - a flag of whether we want the function to publish results of the move to self.base_pub
    # Returns a boolean flag- True when the move succeeded, False if not 
    def move(self, goal, report=True):

        # First clear maps if we have any artefacts from previous motions
        resp = self.costmap_clear()
        #self.space_clear(Empty())
        
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        
        # Allow 45 seconds to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(45)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            self.base_pub.publish("TIMEOUT")
            return False
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                if report:
                    rospy.loginfo("Goal succeeded!")
                    self.base_pub.publish("OK MOVE")
                return True
            else:
                if report:
                    self.base_pub.publish("BAD MOVE")
                return False
    
    def shutdown(self):
        rospy.loginfo("Stopping base controller...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def robot_pose(self, msg):
        rospy.loginfo("Finding robot pose")
        data=""
        #Subscribed coordinate information
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        #Subscribed quaternion information, used to indicate the direction
        orien_z = msg.pose.pose.orientation.z
        orien_w = msg.pose.pose.orientation.w
        
        self.latest_pose_stamped = PoseStamped(msg.header, msg.pose.pose)

        data = "x:" + str(x) + ", y: " + str(y)+ ", z: " + str(orien_z)+ ", w: " + str(orien_w)
        rospy.loginfo(data)

    # This function will publish path cost from the current robot location to the specified target pose,
    # when a target pose is sent to the /base_cntrl/path_cost_qry topic.
    # The result will be published to a topic called /base_cntrl/path_cost
    def publish_path_cost(self, targe_pose):
        # Publish the calculated result
        self.path_cost_pub.publish(self.calculate_path_cost(targe_pose))
                
    # This function will calculate path cost from the current robot location to the specified target pose.
    def calculate_path_cost(self, target_pose):
    
        cost = 0.0
        
        # We can only proceed if current_pose is known
        if (not(self.latest_pose_stamped is None)):
    
            target_pose_stamped = PoseStamped()
            
            # First Header
            target_pose_stamped.header.frame_id = self.latest_pose_stamped.header.frame_id
            target_pose_stamped.header.seq = self.latest_pose_stamped.header.seq
            target_pose_stamped.header.stamp = rospy.Time.now()
            
            # Now the actual pose
            target_pose_stamped.pose = target_pose
            
            # Now let's get the most optimal plan that the current planner can give us
            req = GetPlan()
            req.start = self.latest_pose_stamped
            req.goal = target_pose_stamped
            req.tolerance = .5
            
            response = self.get_plan(req.start, req.goal, req.tolerance)
            #print(response.plan)
            
            # Now that we have the plan, let's go through it and calculate the cost
            prev_pose = None
            for cur_pose in response.plan.poses:
                if (not(prev_pose is None)):
                    # Using Pythagorean theorem to calculate distance between two points
                    cost += sqrt((cur_pose.pose.position.x - prev_pose.pose.position.x) ** 2 + (cur_pose.pose.position.y - prev_pose.pose.position.y) ** 2)
                prev_pose = cur_pose
                
        return cost

    # Calculates path costs to all pre-defined shelf positions and prepares a multiArray object ready to publish
    # if so desired.
    # The return will be a  Float32MultiArray object of the actual path costs in the same order as the shelves.
    def calculate_cost_of_travel(self):
        cost_list = list()

        for shelf_pose in self.shelves:
            cost_list.append(self.calculate_path_cost(shelf_pose))
        
        self.multiArray.data = cost_list
        # return list of cost to travel to each shelf
        return self.multiArray

if __name__ == '__main__':
    try:
        BaseController()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Base Controller terminating.")
