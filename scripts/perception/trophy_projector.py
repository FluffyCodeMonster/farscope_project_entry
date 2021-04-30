#!/usr/bin/env python
# Trophy projection node

'''
Usage
=====
 o Buffering of tf messages:: This node must be started before image_forwarder.py and trophy_detector.py.
'''

# Receive location data (TF - want camera position estimate [together with uncertainty?] and orientation), plus coordinates of bounding box centres.
#  => Custom message type - position, uncertainty, orientation and coordinates of bounding box centres within image.

'''
For each bounding box centre coordinate:
    Compute trophy position in 3D space [on constraint plane]
Package up (3D) coordinates and send to Dawood's code for position estimation

Dawood's code:
    Receive trophy positions
    Combine with existing distributions/point cloud
    Compile into 'trophy list' [maybe I can do this bit]
    Publish to strategy system
'''

'''
Stages of work for prototype:
 o S1: Single plane (e.g. y axis in map frame).
   Requires:
      - TF
      - Projection system
    Store up estimates and publist as a point cloud which Rviz can read.
 o Find trophy planes; add them and bounding coordinates
    - Work out what message type to publish as.
'''

import rospy, sys, tf2_ros
# TODO Temp - need a better message format.
from std_msgs.msg import String
import numpy as np
import math
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud

# Helper function for ProjectionPlane.project(...) code.
def between(num1, num2, test_num):
    # e.g. num1 = 5, num2 = 3.
    if (num1 >= num2):
        return (num2 <= test_num <= num1)
    else:   # if num1 < num2
        return (num1 <= test_num <= num2)

class ProjectionPlane:
    # bounding_coords: list of 4 coordinates - one for each corner.
    def __init__(self, bounding_coords):
        '''
        Specifying the plane:
         o self.n - coordinate of a point on the plane.
         o self.p - normal vector to the plane.
        '''

        self.bl = bounding_coords[0]
        self.tl = bounding_coords[1]
        self.tr = bounding_coords[2]
        self.br = bounding_coords[3]

        # Calculate projection plane from four coordinates
        # Position vector of bottom left corner
        self.p = self.bl
        # TODO Does the orientation of the normal vector matter?
        u = self.tl - self.p
        v = self.br - self.p
        # Calculate normal vector
        self.n = np.cross(u, v)
        # Normalise
        self.n = self.n / np.linalg.norm(self.n)

    '''
    Compute intersection of line of sight (through trophy) and projection plane modelled by this object.

    ray_origin_vec: position vector of camera
    ray_dir_vec: vector of ray direction from camera

    returns: [found (True/False), numpy array of intersection coordinates (or None if not found)]
    '''
    # TODO Is -1 the correct thing to return - should it be returning None (etc) instead?
    # TODO [IMPORTANT] Need to check to see if it is within the confines of the shelf (have a look at Arturs' code).
    def project(self, ray_origin_vec, ray_dir_vec):
        intersect_coord = None

        # Try-catch block in case trophy projection ray and trophy plane are parallel - divide by zero exception could occur.
        try:
            # Finding line parameter value for point of intersection.
            # TODO How to handle if this intersection doesn't exist - e.g. parallel lines?
            dist = (np.dot((self.p - ray_origin_vec), self.n)) / np.dot(ray_dir_vec, self.n)

            # Find the position of intersection in 3D space (in the map frame).
            intersect_coord = ray_origin_vec + dist*ray_dir_vec
        except Exception as error:
            # TODO Need to handle this error properly. Also, should it be error.message?
            print("Error: Could not perform trophy projection. Message: {}".format(error))
            return [False, None]

        # Check to see if it falls within the shelf boundaries and is *in front of* the lens.
        # See if projection point lies in shelving unit trophy plane: x constraints, y constraints, z constraints + must be forward projection (not behind camera).
        if (between(self.bl[0], self.tr[0], intersect_coord[0]) and between(self.bl[1], self.tr[1], intersect_coord[1]) and between(self.bl[2], self.tr[2], intersect_coord[2]) and (dist >= 0)):
            return [True, intersect_coord]
        else:
            return [False, None]

'''
centre: position vector of camera
cam_orientation: direction vector of camera view/perspective
'''

# Calculate direction vector of line through trophy centre on camera projective plane.
def dir_vec_from_image(image_dims, centre):
    # Calculate the projection vector relative to the orientation vector supplied.

    # print("Dims: {}".format(image_dims))    # Testing
    
    # centre_image_frame_coord gets recalculated with every image received - not the most efficient, but at least it allows for changes in image size.
    # NOTE: If camera field of view changes, have to adjust it manually (see global variables below).
    # CAUTION: image_dims = (height, width) - not the way round you'd expect!
    image_height = image_dims[0]
    image_width = image_dims[1]
    image_centrepoint = (round(float(image_width) / 2), round(float(image_height) / 2))

    # Define +ve to right, +ve upwards.
    # (0,0) is at the top-left of the images.
    x_disp = centre[0] - image_centrepoint[0]
    y_disp = image_centrepoint[1] - centre[1]
    print("Centre: {}".format(centre))
    print("(x_rel, y_rel) = ({}, {})".format(x_disp, y_disp))

    # I'm not sure if this is actually the focal distance, by definition.
    # TODO Should do this separately for x and y.
    ##focal_distance = (2 / image_width) * math.tan(CAM_FOV_RADIANS / 2)

    # Work out angle. Camera has no distortion (see the file TODO [complete this]), so this is a linear interpolation.
    # image_dims = (height, width)
    x_angle = ( x_disp / image_width ) * CAM_FOV_RADIANS
    y_angle = ( y_disp / image_height ) * CAM_FOV_RADIANS
    ##x_angle = math.atan(x_disp / focal_distance)
    ##y_angle = math.atan(y_disp / focal_distance)

    # Assemble into a unit vector.
    # Angles in radians.
    # -1 for y coordinate because of ROS RH frame coordinate system.
    projn_vec_cam_frame = np.array([1, -1*math.tan(x_angle), math.tan(y_angle)])
    print("x angle: {}, y angle: {}".format(x_angle, y_angle))
    # Normalise
    projn_vec_cam_frame = projn_vec_cam_frame / np.linalg.norm(projn_vec_cam_frame)

    print(projn_vec_cam_frame)

    return projn_vec_cam_frame

def get_cam_posn(time):
    success = False

    # Point of camera origin.
    cam_origin = PointStamped()
    # TODO Should this be 'time'?
    cam_origin.header.stamp = time
    cam_origin.header.frame_id = 'camera1_lens'
    cam_origin.point.x = 0
    cam_origin.point.y = 0
    cam_origin.point.z = 0

    print("Created camera1 origin for transformation.")

    try:
        # TODO Need to deal with if time is 0 because the clock hasn't been published yet(?)? - http://wiki.ros.org/roscpp/Overview/Time
    
        # Now need to transform origin and position vector of one.
        # Times out after 1 second. What happens if the buffer doesn't contain the
        # transformation after this duration? [I think it throws an error]
        # --> Even works with a duration of 0.001 - how does this work? TODO
        cam_in_map = tf_buffer.transform(cam_origin, 'map', rospy.Duration(1))
        print("Performed transform")

        cam_posn_vec = np.array([cam_in_map.point.x, cam_in_map.point.y, cam_in_map.point.z])

        success = True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        # TODO Need to add more details to this. Might just be doing it because the buffer isn't large enough yet.
        # TODO Might need to revise this error message now that I've chopped things around.
        print("Transform error ~ it's likely that the picture time predates the tf transform buffer. Image ignored.")
        print("   Message:: {}".format(e))
    
    # cam_posn_vec is a numpy 3-vector
    return [success, cam_posn_vec]

# projn_vec_cam_frame, cam_origin should be numpy arrays.
def get_projn_vector(time, projn_vec_cam_frame, cam_origin):
    success = False

    print("Created projection vector points for transformation.")

    # We consider a unit projection vector in the camera1_lens frame
    # (position vector in direction of trophy from camera1_lens origin, of length one),
    # transform it into the map frame, and then subtract off the position of
    # the camera to get the projection vector in the map frame.

    # Projection vector in camera1_lens frame.
    projn_vec_cam_frame_point = PointStamped()
    # TODO Should this be 'time'?
    projn_vec_cam_frame_point.header.stamp = time
    projn_vec_cam_frame_point.header.frame_id = 'camera1_lens'
    projn_vec_cam_frame_point.point.x = projn_vec_cam_frame[0]
    # y -ve because left handed coordinate frame.
    projn_vec_cam_frame_point.point.y = projn_vec_cam_frame[1]
    projn_vec_cam_frame_point.point.z = projn_vec_cam_frame[2]

    try:
        # TODO Need to deal with if time is 0 because the clock hasn't been published yet(?)? - http://wiki.ros.org/roscpp/Overview/Time
    
        # Now need to transform origin and position vector of one.
        # Times out after 1 second. What happens if the buffer doesn't contain the
        # transformation after this duration? [I think it throws an error]
        # --> Even works with a duration of 0.001 - how does this work? TODO
        projn_vec_map_frame_point = tf_buffer.transform(projn_vec_cam_frame_point, 'map', rospy.Duration(1))
        print("Performed transform")

        projn_vec_map = np.array([projn_vec_map_frame_point.point.x, projn_vec_map_frame_point.point.y, projn_vec_map_frame_point.point.z])

        success = True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        # TODO Need to add more details to this. Might just be doing it because the buffer isn't large enough yet.
        # TODO Might need to revise this error message now that I've chopped things around.
        print("Transform error ~ it's likely that the picture time predates the tf transform buffer. Image ignored.")
        print("   Message:: {}".format(e))
    
    # Get position vector in map frame by subtracting off position of camera in map frame.
    projn_vec = projn_vec_map - cam_origin

    # 'Projection ray' direction as a numpy 3-vector.
    return [success, projn_vec]

def on_centres(centres_msg):
    success = True

    [image_time, image_dims, centres] = parse_centres_string(centres_msg.data)

    # Testing
    # print(image_time)
    # print(centres)
    trophy_coords = []
    
    # Get camera position in 'map' frame, as well as frame orientation (i.e. direction vector through (1,0,0)
    # from the camera1 lens), both as numpy 3d-vectors.
    # cam_posn_transform_success
    [success, cam_coord] = get_cam_posn(image_time)
    print("Camera coordinate: {}".format(cam_coord))
    #print(cam_orientation)

    if success:
        # Perform projection. For each centre:
        for centre in centres:
            print("Projecting centre")
            # Get projection vector in camera lens frame.
            projn_vec_in_cam_frame = dir_vec_from_image(image_dims, centre)
            # Convert into a projection direction in the map frame.
            # projn_vec_transform_success
            [success, projection_line_dir] = get_projn_vector(image_time, projn_vec_in_cam_frame, cam_coord)
            print("Projn vector in map frame: {}".format(projection_line_dir))
            
            if success:
                # Iterate through planes and perform projections.
                for plane in planes:
                    [projection_success, trophy_coord] = plane.project(cam_coord, projection_line_dir)
                    if projection_success:
                        trophy_coords.append(trophy_coord)
                        # Break out of inner for loop for this centre, since the projection plane has already been found.
                        break
                # TODO Flag a warning if a trophy couldn't be projected.
    
    # TODO Need to check this error handling mechanism.
    if success:
        for coord in trophy_coords:
            print("Coord: {}".format(coord))

        # Publish new centres to trophy estimate clustering code:
        coord_pub.publish(gen_msg_string(trophy_coords))

        # For testing - publish point cloud:
        if testing:
            point_cloud.points = []
            for coords in trophy_coords:
                point = Point()
                point.x = coords[0]
                point.y = coords[1]
                point.z = coords[2]

                # Add to point cloud.
                point_cloud.points.append(point)
            
            # Publish point cloud.
            point_cloud.header.stamp = rospy.Time.now()
            point_cloud_pub.publish(point_cloud)

# TODO Temp - need to develop a custom message format
def gen_msg_string(centres):
    time = rospy.Time.now()
    msg_string = "{}.{}".format(time.secs, time.nsecs)
    for centre in centres:
        # Cannot use '.' to delimit as these are floats. Have to use ':' instead.
        msg_string += ";{}:{}:{}".format(centre[0], centre[1], centre[2])
    return msg_string

def parse_centres_string(centres_string):
    # String format:
    # secs.nsecs;image_height.image_width;x1.y1;x2.y2;...
    centres = []
    msg_split = centres_string.split(';')
    print(msg_split)

    # Parsing time.
    dot_split = msg_split[0].split('.')
    print(dot_split)
    time = rospy.Time(int(dot_split[0]), int(dot_split[1]))

    # Parsing image dimensions.
    dot_split = msg_split[1].split('.')
    print(dot_split)
    image_dims = (int(dot_split[0]), int(dot_split[1]))

    # Parsing centre coords.
    for centre_string in msg_split[2:]:
        # TODO Should this be double bracketed?
        dot_split = centre_string.split('.')
        print(dot_split)
        centres.append((int(dot_split[0]), int(dot_split[1])))
    
    return [time, image_dims, centres]

# xy_centre needs to be on the trophy plane.
# orientation_angle as it is in the world file - 0 means that the shelf is pointing in the
# -ve y direction, -pi/2 means that it is pointing in the -ve x direction, etc.
#    [angle in radians]
def get_shelf_boundaries(xy_centre, orientation_angle):
    # TODO [CRITICAL] I wonder if a shelf is more than 0.88 wide - more like 1 in width? Maybe change 0.44 to 0.5.
    side_offset = 0.44 * np.array([-math.cos(orientation_angle), -math.sin(orientation_angle), 0])
    print(side_offset)
    print("x: {}, y: {}, z: {}".format(side_offset[0], side_offset[1], side_offset[2]))

    # bottom left, top left, top right, bottom right (vertices, clockwise)
    xy_centre_bottom_array = np.array([xy_centre[0], xy_centre[1], 0])
    print(xy_centre_bottom_array)
    bl = xy_centre_bottom_array + side_offset
    br = xy_centre_bottom_array - side_offset
    print(-side_offset)
    print(br[1])
    tr = br + np.array([0, 0, 1.56])
    tl = bl + np.array([0, 0, 1.56])
    return [bl, tl, tr, br]

# Set to True to store up trophy position estimates and output them to Rviz as a point cloud.
if  '-t' in sys.argv:
    testing = True
    print("*** Testing mode: Sending trophy coordinates to Rviz as point cloud ***")
else:
    testing = False
    print("*** Run mode: Sending trophy estimates to clustering node ***")

# Testing
# print(get_shelf_boundaries((2, 0), -math.pi / 2))

rospy.init_node('trophy_projector')

######### Set up global variables and constants #########

CAM_FOV_RADIANS = 1.3962634 # camera1 field of view

# Set up planes (TODO read coordinates from file?):
# List of projection planes.
# Bottom-left, TL, TR, BR (clockwise from bottom left)
# Coordinate system - right handed frame.
# NOTE These are numpy arrays.
planes = [
    # Shelf 1
    #ProjectionPlane([(2, -0.44, 0), (2, -0.44, 1.56), (2, 0.44, 1.56), (2, 0.44, 0)]),
    # TODO Do I need the double brackets here?
    ProjectionPlane(get_shelf_boundaries((2, 0), -math.pi / 2)),
    # Shelf 2
    ProjectionPlane(get_shelf_boundaries((2, -1.5), -math.pi / 2)),
    # Shelf 3
    ProjectionPlane(get_shelf_boundaries((2, -3), -math.pi / 2)),
    # Shelf 4
    ProjectionPlane(get_shelf_boundaries((1, -4), -math.pi)),
    #Shelf 5
    ProjectionPlane(get_shelf_boundaries((-0.5, -4), -math.pi)),
    # Shelf 6
    ProjectionPlane(get_shelf_boundaries((-2, -4), -math.pi)),
    # Shelf 7
    ProjectionPlane(get_shelf_boundaries((-3, -3), math.pi / 2)),
    # Shelf 8
    ProjectionPlane(get_shelf_boundaries((-2, -1), 0))
    ]

# Stores trophy coords determined.
# Note: if a centre does not project onto a projection plane (i.e. one of the shevles), it will not be stored.
trophy_coords = []

if testing:
    point_cloud = PointCloud()
    point_cloud.header.frame_id = "map"
    point_cloud.points = []
    point_cloud.channels = []

# Start tf buffer and listener
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

#########################################################

# TODO Need to define the correct message types for Msg1 and Msg2.
# Message coming from trophy detector. Will need:
# {(Camera pose, coordinates in frame (x, y)), ...}
# TODO Currently reading in as a string, but this is only a temporary solution.
coord_sub = rospy.Subscriber("detected_trophy_centres", String, on_centres)

if testing:
    # Send point cloud to Rviz.
    point_cloud_pub = rospy.Publisher("trophy_projections_raw_point_cloud", PointCloud)

# Send list of new trophy coordinate estimates to 'clustering' node.
# TODO Define a custom message type - sending as String is a temporary solution! 
coord_pub = rospy.Publisher("trophy_coord_ests_3d", String)

# TODO Do I need the while, or just the spin()? Check that I am doing this correctly.
while not rospy.is_shutdown():
    rospy.spin()

# Need:
# o ROS topic for sending 2D coordinates of bounding box centres in images.
# Point cloud ROS topic.