import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import pi
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient

# Shelf unit locations: 
# WRT orientation we only care if the shelf is facing west, east, north or south as there are no other angles. 
# 0: west; 1: east; 2: north; 3: south 
#                   x         y      orientation. 
shelf_unit_loc = [[2.0,      0.0,      0],
                  [2.0,     -1.5,      0],
                  [2.0,     -3.0,      0],
                  [1.0,     -4.0,      2],
                  [-0.5,    -4.0,      2],
                  [-2.0,    -4.0,      2],
                  [-3.0,    -3.0,      1],
                  [-2.0,    -1.0,      3]]

shelf_unit_height = 1.2
shelf_width = 0.88 # width of the shelf : X
shelf_depth = 0.4 # depth of the shelf : Y
shelf_height = 0.36 # height of the shelf within the unit - not including the floor and ceiling of the shelf
shelf_unit_foot_thickness = 0.06
shelf_floor_thickness = 0.02

# According to the world: shelf_heights: [0.06, 0.43, 0.81, 1.21] # do not edit - must match world file
# But my maths says otherwise: [0.06, 0.44, 0.82, 1.20]. It's probably not hugely important because we'll
# introduce variance of half of the shelf_height.
shelf_levels = [shelf_unit_foot_thickness, 
                shelf_unit_foot_thickness + 1 * shelf_height + 1 * shelf_floor_thickness,
                shelf_unit_foot_thickness + 2 * shelf_height + 2 * shelf_floor_thickness,
                shelf_unit_foot_thickness + 3 * shelf_height + 3 * shelf_floor_thickness] # heights at which the shelves in the unit start. This is in the order of: bottom, low, high, top.

def where_is_this_trophy(trophy_coords):
    trophy_x, trophy_y, trophy_z = trophy_coords
        
    found = False
    # Now let's go through all of the shelf units and find if the trophy is within that unit.
    # If the shelf unit is facing east or west, then we want to give trophy Y coordinate the variance of a shelf width around the Y coordinate of the shelf unit.
    # And the X coordinate then will have the variance of shelf depth.
    # 
    # If the shelf unit is facing north or south, then we want to give trophy X coordinate the variance of a shelf width around the X coordinate of the shelf unit.
    # And the Y coordinate then will have the variance of shelf depth.
    shelf_id = 0 # ID of identified shelf
    position_on_shelf = 0.0
    for cur_loc in shelf_unit_loc:
        shelf_id += 1
        
        # First find out which way the shelf is facing and therefore what are the variances
        if cur_loc[2] <= 1:
            x_variance = shelf_depth / 2.0
            y_variance = shelf_width / 2.0
        else:
            x_variance = shelf_width / 2.0
            y_variance = shelf_depth / 2.0
            
        # Now find out which shelf unit we reside.
        if (abs(cur_loc[0] - trophy_x) <= x_variance) and (abs(cur_loc[1] - trophy_y) <= y_variance):
            if cur_loc[2] == 0:
                position_on_shelf = (cur_loc[1] - trophy_y) / y_variance 
            elif cur_loc[2] == 1:
                position_on_shelf = -1.0 * (cur_loc[1] - trophy_y) / y_variance
            elif cur_loc[2] == 2:
                position_on_shelf = (cur_loc[0] - trophy_x) / x_variance
            elif cur_loc[2] == 3:
                position_on_shelf = -1.0 * (cur_loc[0] - trophy_x) / x_variance
                
            found = True
            break
            
    # If the shelf was not found, then resetting the variables
    if not found:
        position_on_shelf = 0.0
        shelf_id = 0
    
    found = False
    
    # Now find out how high - which level is our trophy on
    level_id = 0 # Which shelf in the shelf unit does the trophy reside on
    for level in shelf_levels:
        if abs(trophy_z - level) <= shelf_height / 2.0:
            found = True
            break

        level_id += 1

    # If the level was not found, then resetting the variable
    if not found:
        level_id = 0

    return (shelf_id, level_id, position_on_shelf, trophy_coords)

def where_are_these_trophies(trophy_coord_list):
    result = list()
    for tc in trophy_coord_list:
        result.append(where_is_this_trophy(tc))
    
    return result

class GripperController:
    """Control the FARSCOPE simple gripper model"""
    def __init__(self):
        self.pub1 = rospy.Publisher('finger1_controller/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('finger2_controller/command', Float64, queue_size=10)

    def move(self, cmd_in=0.0):
        rospy.loginfo('Sending {} to both finger controllers.'.format(cmd_in))
        r = rospy.Rate(10)
        for ii in range(5):
            r.sleep()
            self.pub1.publish(cmd_in)
            self.pub2.publish(cmd_in)

    def open(self):
        self.move(-0.04)

    def close(self):
        self.move(0.04)


class BaseDriver:
    """Utility for moving the mobile base"""
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move(self, x_trans_in, y_trans_in=0.0, z_rot_in=0.0, duration_in=1.0):
        cmd = Twist()
        cmd.linear.x = x_trans_in
        cmd.linear.y = y_trans_in
        cmd.angular.z = z_rot_in
        rospy.loginfo('Sending dX={}, dY={}, dT={} for {}s'.format(x_trans_in,
                                                                   y_trans_in,
                                                                   z_rot_in,
                                                                   duration_in))
        r = rospy.Rate(10)
        for ii in range(int(10*duration_in)):
            r.sleep()
            self.pub.publish(cmd)
        rospy.loginfo('Stopping')
        cmd = Twist()
        for ii in range(10):
            r.sleep()
            self.pub.publish(cmd)


class ArmMover:
    """Client to move the robot arm"""
    def __init__(self):
        self.client = SimpleActionClient("/arm_controller/follow_joint_trajectory",
                                         FollowJointTrajectoryAction)
        rospy.loginfo("waiting to connect...")
        self.client.wait_for_server()
        rospy.loginfo("connected! ")
        self.joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'elbow_joint',
                            'wrist_1_joint',
                            'wrist_2_joint',
                            'wrist_3_joint']

    def move(self,
             shoulder_lift_cmd_in=-pi/4.0,
             elbow_cmd_in=pi/2.0,
             wrist_2_cmd = 0.5*pi,  # straight along arm
             duration_in=5.0):
        wrist_1_cmd = (-1.0)*(shoulder_lift_cmd_in + elbow_cmd_in)  # keep it level
        #wrist_2_cmd = 0.5*pi  # straight along arm
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_names
        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, shoulder_lift_cmd_in, elbow_cmd_in, wrist_1_cmd, wrist_2_cmd, 0.0]
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p1.time_from_start = rospy.Duration(duration_in)
        g.trajectory.points.append(p1)
        self.client.send_goal(g)
        rospy.loginfo("Sent goal and waiting for arm")
        self.client.wait_for_result()
        rospy.loginfo("Arm move complete")
