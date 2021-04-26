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

class ProjectionPlane:
    # bounding_coords: list of 4 coordinates - one for each corner.
    def __init__(self, bounding_coords):
        '''
        Specifying the plane:
         o self.n - coordinate of a point on the plane.
         o self.p - normal vector to the plane.
        '''

        # Calculate projection plane from four coordinates
        # Position vector of bottom left corner
        self.p = np.array(bounding_coords[0])
        # TODO Does the orientation of the normal vector matter?
        u = np.array(bounding_coords[1]) - self.p
        v = np.array(bounding_coords[3]) - self.p
        # Calculate normal vector
        self.n = np.cross(u, v)
        # Normalise
        self.n = self.n / np.linalg.norm(self.n)

    '''
    Compute intersection of line of sight (through trophy) and projection plane modelled by this object.

    ray_origin_vec: position vector of camera
    ray_dir_vec: vector of ray direction from camera

    returns: -1 if no intersection, else tuple of intersection coordinates.
    '''
    # TODO Is -1 the correct thing to return - should it be returning None (etc) instead?
    # TODO [IMPORTANT] Need to check to see if it is within the confines of the shelf (have a look at Arturs' code).
    def project(self, ray_origin_vec, ray_dir_vec):
        # Finding line parameter value for point of intersection.
        # TODO How to handle if this intersection doesn't exist - e.g. parallel lines?
        dist = (np.dot((self.p - ray_origin_vec), self.n)) / np.dot(ray_dir_vec, self.n)

        # Find the position of intersection in 3D space (in the map frame).
        intersect_coord = ray_origin_vec + dist*ray_dir_vec

        # TODO check to see if it falls within the shelf boundaries...
        # ... and is *in front of* the lens.
        found = True
        
        return [found, intersect_coord]

'''
centre: position vector of camera
cam_orientation: direction vector of camera view/perspective
'''

# Calculate direction vector of line through trophy centre on camera projective plane.
def find_line_dir_vec(image_dims, cam_orientation_vec, centre):
    # Calculate the projection vector relative to the orientation vector supplied.

    # print("Dims: {}".format(image_dims))    # Testing
    
    # centre_image_frame_coord gets recalculated with every image received - not the most efficient, but at least it allows for changes in image size.
    # NOTE: If camera field of view changes, have to adjust it manually (see global variables below).
    image_centrepoint = (round(float(image_dims[0]) / 2), round(float(image_dims[1]) / 2))

    # Define +ve to right, +ve upwards.
    # (0,0) is at the top-left of the images.
    x_disp = centre[1] - image_centrepoint[1]
    y_disp = centre[0] - image_centrepoint[0]

    # Work out angle. Camera has no distortion (see the file TODO [complete this]), so this is a linear interpolation.
    # image_dims = (height, width)
    x_angle = ( x_disp / image_dims[1] ) * CAM_FOV_RADIANS
    y_angle = ( y_disp / image_dims[0] ) * CAM_FOV_RADIANS

    # Angles in radians
    projn_vec_cam_frame = np.array([1, -1*math.tan(x_angle), math.tan(y_angle)])

    # Add to camera orientation frame vector to get projection direction vector in map frame.
    line_dir_vec = cam_orientation_vec + projn_vec_cam_frame

    print(line_dir_vec)

    return line_dir_vec

def get_cam_posn_and_orientation(time):
    success = False

    # Point of camera origin.
    cam_origin = PointStamped()
    cam_origin.header.stamp = rospy.Time.now()
    cam_origin.header.frame_id = 'camera1_lens'
    cam_origin.point.x = 0
    cam_origin.point.y = 0
    cam_origin.point.z = 0

    # Point one out (x-direction) from camera origin.
    x1_cam = PointStamped()
    x1_cam.header.stamp = rospy.Time.now()
    x1_cam.header.frame_id = 'camera1_lens'
    x1_cam.point.x = 1
    x1_cam.point.y = 0
    x1_cam.point.z = 0

    print("Created camera1 origin and direction points for transformation.")

    try:
        # TODO Need to deal with if time is 0 because the clock hasn't been published yet(?)? - http://wiki.ros.org/roscpp/Overview/Time
    
        # Now need to transform origin and position vector of one.
        # Times out after 1 second. What happens if the buffer doesn't contain the
        # transformation after this duration? [I think it throws an error]
        # --> Even works with a duration of 0.001 - how does this work? TODO
        cam_in_map = tf_buffer.transform(cam_origin, 'map', rospy.Duration(1))
        x1_map = tf_buffer.transform(x1_cam, 'map', rospy.Duration(1))
        print("Performed transforms")

        cam_posn_vec = np.array([cam_in_map.point.x, cam_in_map.point.y, cam_in_map.point.z])
        x1_posn = np.array([x1_map.point.x, x1_map.point.y, x1_map.point.z])
        projn_vec = x1_posn - cam_posn_vec

        success = True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        # TODO Need to add more details to this. Might just be doing it because the buffer isn't large enough yet.
        print("Transform error ~ it's likely that the picture time predates the tf transform buffer. Image ignored.")
        print("   Message:: {}".format(e))
    
    # numpy 3-vectors - camera1 position and 'projection ray' direction.
    return [success, cam_posn_vec, projn_vec]

def on_centres(centres_msg):
    [image_time, image_dims, centres] = parse_centres_string(centres_msg.data)

    # Testing
    # print(image_time)
    # print(centres)
    trophy_coords = []
    
    # Get camera position in 'map' frame, as well as frame orientation (i.e. direction vector through (1,0,0)
    # from the camera1 lens), both as numpy 3d-vectors.
    [transform_success, cam_coord, cam_orientation] = get_cam_posn_and_orientation(image_time)
    #print(cam_coord)
    #print(cam_orientation)

    if transform_success:
        # Perform projection. For each centre:
        for centre in centres:
            print("Projecting centre")
            projection_line_dir = find_line_dir_vec(image_dims, cam_orientation, centre)
            # Iterate through planes and perform projections.
            for plane in planes:
                [projection_success, trophy_coord] = plane.project(cam_coord, projection_line_dir)
                if projection_success:
                    trophy_coords.append(trophy_coord)
                    # Break out of inner for loop for this centre, since the projection plane has already been found.
                    break
    
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

# Set to True to store up trophy position estimates and output them to Rviz as a point cloud.
if  '-t' in sys.argv:
    testing = True
    print("*** Testing mode: Sending trophy coordinates to Rviz as point cloud ***")
else:
    testing = False
    print("*** Run mode: Sending trophy estimates to clustering node ***")

rospy.init_node('trophy_projector')

######### Set up global variables and constants #########

CAM_FOV_RADIANS = 1.3962634 # camera1 field of view

# Set up planes (TODO read coordinates from file?):
# List of projection planes.
# Bottom-left, TL, TR, BR (clockwise from bottom left)
# Coordinate system - right handed frame.
planes = [
    # Shelf 1
    ProjectionPlane([(2, -0.44, 0), (2, -0.44, 1.56), (2, 0.44, 1.56), (2, 0.44, 0)])
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