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

class RosThread (threading.Thread):
    def __init__(self, request_name_arg, ros_topic_arg, ros_type_arg, callback_function_arg, queue_size_arg):
        threading.Thread.__init__(self)
        self._request_name = request_name_arg
        self._ros_topic = ros_topic_arg
        self._ros_type = ros_type_arg
        self._callback_function = callback_function_arg
        self._queue_size_arg = queue_size_arg
    
    def run(self):
        print('Starting request listener thread: {}'.format(self._request_name))
        request_sub = rospy.Subscriber(self._ros_topic, self._ros_type, self._callback_function, queue_size = self._queue_size_arg)

def on_camera1_image(image):
    global image_requested_from_detector
    global camera1_image_requested_from_robot

    # If a new camera1 image has been requested (from trophy detector and robot):
    if image_requested_from_detector and camera1_image_requested_from_robot:
        image_pub.publish(image)
        image_requested_from_detector = False
        camera1_image_requested_from_robot = False
        print('Image published to trophy detector')

def on_camera2_image(image):
    global image_requested_from_detector
    global camera2_image_requested_from_robot

    # If a new camera2 image has been requested (from trophy detector and robot):
    if image_requested_from_detector and camera2_image_requested_from_robot:
        image_pub.publish(image)
        image_requested_from_detector = False
        camera2_image_requested_from_robot = False
        print('Image published to trophy detector')

def on_detector_request(request):
    global image_requested_from_detector
    print("Image request from detector received")
    image_requested_from_detector = True

def on_robot_request(request_string):
    global camera1_image_requested_from_robot
    global camera2_image_requested_from_robot

    if (request_string.data == "Image_request::camera1"):
        print("Camera1 image request received from robot")
        camera1_image_requested_from_robot = True
    elif (request_string.data == "Image_request::camera2"):
        print("Camera2 image request received from robot")
        camera2_image_requested_from_robot = True

# Signals (when True) that trophy detector is ready to receive another image.
image_requested_from_detector = False
camera1_image_requested_from_robot = False
camera2_image_requested_from_robot = False

rospy.init_node('image_forwarder')
# image_sub = rospy.Subscriber("camera1/image_raw", Image, on_image, queue_size=1)
image_pub = rospy.Publisher("image_for_trophy_detection", Image)

# Start trophy detector request listener thread:
trophy_detector_request_listener = RosThread("trophy detector", "trophy_image_request", Empty, on_detector_request, 1)
trophy_detector_request_listener.start()
# Start robot image request listener thread:
robot_image_request_listener = RosThread("robot listener", "/trophy_image_robot_request_response", String, on_robot_request, 1)
robot_image_request_listener.start()

# Start camera1 and camera2 image threads:
camera1_image_processor = RosThread("camera1 image processor", "camera1/image_raw", Image, on_camera1_image, 1)
camera2_image_processor = RosThread("camera2 image processor", "camera2/image_raw", Image, on_camera2_image, 1)
camera1_image_processor.start()
camera2_image_processor.start()

# Keep alive
while not rospy.is_shutdown():
    rospy.spin()