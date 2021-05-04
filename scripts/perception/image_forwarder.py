#!/usr/bin/env python
# FT 12/04/21
# Forwards images from the camera to the YOLO detector on request (so that YOLO can keep up).

import threading
import sys, rospy
from sensor_msgs.msg import Image
# Use an empty message to send a signal.
from std_msgs.msg import Empty, String


# TODO Will these two block each other? Do I need multithreading? Could multithreading lead
# to a race condition s.t. a request is missed?

class RequestListener (threading.Thread):
    def __init__(self, request_name, ros_topic, ros_type, callback_function):
        threading.Thread.__init__(self)
        self._request_name = request_name
        self._ros_topic = ros_topic
        self._ros_type = ros_type
        self._callback_function = callback_function
    
    def run(self):
        print('Starting request listener thread: {}'.format(self._request_name))
        request_sub = rospy.Subscriber(self._ros_topic, self._ros_type, self._callback_function)

def on_image(image):
    global image_requested_from_detector
    global image_requested_from_robot
    # If a new image has been requested (from trophy detector and robot):
    if image_requested_from_detector and image_requested_from_robot:
        image_pub.publish(image)
        image_requested_from_detector = False
        image_requested_from_robot = False
        print('Image published to trophy detector')

def on_detector_request(request):
    global image_requested_from_detector
    print("Request received")
    image_requested_from_detector = True

def on_robot_request(request_string):
    global image_requested_from_robot
    if (request_string.data == "Image_request"):
        print("Image request received from robot")
        image_requested_from_robot = True

image_requested_from_detector = False
image_requested_from_robot = False

rospy.init_node('image_forwarder')
image_sub = rospy.Subscriber("camera1/image_raw", Image, on_image, queue_size=1)
image_pub = rospy.Publisher("image_for_trophy_detection", Image)

# Start trophy detector request listener thread:
trophy_detector_request_listener = RequestListener("trophy detector", "trophy_image_request", Empty, on_detector_request)
trophy_detector_request_listener.start()
# Start robot image request listener thread:
robot_image_request_listener = RequestListener("robot listener", "/trophy_image_robot_request_response", String, on_robot_request)
robot_image_request_listener.start()

# Keep alive
while not rospy.is_shutdown():
    rospy.spin()