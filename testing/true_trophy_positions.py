# Publishes the true trophy positions for comparison with those derived by the perception system.

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud

shelf_bases = [0.06, 0.44, 0.81, 1.2]
trophy_height = 0.08

# shelf: 0 - 3
def add_point(cloud_points, x, y, shelf):
    point = Point()

    point.x = x
    point.y = y
    point.z = shelf_bases[shelf] + trophy_height

    cloud_points.append(point)


def setup_cloud():
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = "odom"

    # Add points to cloud.
    points = []
    for shelf in [0, 1, 2, 3]:
        # Shelf 1
        add_point(points, 2, 0, shelf)
        # Shelf 2
        add_point(points, 2, -1.5, shelf)
        # Shelf 3
        add_point(points, 2, -3, shelf)
        # Shelf 4
        add_point(points, 1, -4, shelf)
        # Shelf 5
        add_point(points, -0.5, -4, shelf)
        # Shelf 6
        add_point(points, -2, -4, shelf)
        # Shelf 7
        add_point(points, -3, -3, shelf)
        # Shelf 8
        add_point(points, -2, -1, shelf)

    cloud.points = points
    return cloud


rospy.init_node('true_trophy_positions')
# Set up publisher
true_point_pub = rospy.Publisher('/true_trophy_cloud', PointCloud)
true_cloud = setup_cloud()

while True:
    input(" > Press enter to publish")
    true_point_pub.publish(true_cloud)

while not rospy.is_shutdown():
    rospy.spin()