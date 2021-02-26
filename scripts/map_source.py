#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

# We will have this width and height of the map:
map_width = 15.0
map_height = 10.0
map_seq = 0 # sequence of the map. This will have to increase with each issue

# Create a dummy map
def create_og():
    # Create a dummy occupancy grid
    og = OccupancyGrid()

    map_seq+=1

    # First fill in the header
    og.header.stamp = rospy.Time.now()
    og.header.frame_id = "map"
    og.header.seq = map_seq

    # Now meta data
    og.info.resolution = 1 # 1m per cell on the occupancy grid
    og.info.width = (int)map_width # width
    og.info.height = (int)map_height # height
    og.info.map_load_time = rospy.Time.now()

    # We will put ourselves into the top left corner.
    # Rotated maps are not supported... quaternion represents no rotation. 
    og.info.origin = Pose(Point(-1 * (map_width / 2.0), -1 * (map_height / 2.0), 0),
                           Quaternion(0, 0, 0, 1))
                           
    # And finally the actual occupancy grid in a flattened form
    grid = np.zeros((map_height, map_width))
    flat_grid = grid.reshape((grid.size,)) * 100
    og.data = list(np.round(flat_grid))
    
    return og

# Now that we know how to create the message, let's create a topic and publish it
rospy.init_node("map_source")
map_pub = rospy.Publisher("/map_source/map", OccupancyGrid, latch=True)
map_metadata_pub = rospy.Publisher('/map_source/metadata', MapMetaData, latch=True)

rate = rospy.Rate(2.0)
while not rospy.is_shutdown():
    rate.sleep()
    map_pub.publish(create_og())
    
