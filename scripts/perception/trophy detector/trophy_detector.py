#!/usr/bin/env python
# Code based on https://opencv-tutorial.readthedocs.io/en/latest/yolo/yolo.html

# YOLO object detection
import sys, rospy
import cv2 as cv
import numpy as np
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import String
from cv_bridge import CvBridge

def setup_yolo():
    # TODO Remove globals:
    global net, ln, colors, classes
    ###WHITE = (255, 255, 255)

    # Load names of classes and get random colors
    classes = open('classes.names').read().strip().split('\n')
    np.random.seed(42)
    colors = np.random.randint(0, 255, size=(len(classes), 3), dtype='uint8')

    # Give the configuration and weight files for the model and load the network.
    net = cv.dnn.readNetFromDarknet('yolov3.cfg', 'yolov3.weights')
    net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
    # net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

    # determine the output layer
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

def run_network(image):    
    # TODO Ensure this is the size that the network is expecting - 608x608?
    blob = cv.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)

    net.setInput(blob)
    outputs = net.forward(ln)

    # combine the 3 output groups into 1 (10647, 85)
    # large objects (507, 85)
    # medium objects (2028, 85)
    # small objects (8112, 85)
    outputs = np.vstack(outputs)

    return post_process(image, outputs, 0.5)

def post_process(img, outputs, conf):
    H, W = img.shape[:2]

    boxes = []
    confidences = []
    classIDs = []

    # Contains image coordinates of trophy centres.
    centres = []

    for output in outputs:
        scores = output[5:]
        classID = np.argmax(scores)
        confidence = scores[classID]
        if confidence > conf:
            x, y, w, h = output[:4] * np.array([W, H, W, H])
            p0 = int(x - w//2), int(y - h//2)
            p1 = int(x + w//2), int(y + h//2)
            boxes.append([*p0, int(w), int(h)])
            confidences.append(float(confidence))
            classIDs.append(classID)
            # cv.rectangle(img, p0, p1, WHITE, 1)

    indices = cv.dnn.NMSBoxes(boxes, confidences, conf, conf-0.1)
    if len(indices) > 0:
        for i in indices.flatten():
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])

            # Testing
            print("x: {}, y: {}, w: {}, h: {}".format(x, y, w, h))

            # Determining centres.
            # TODO int(round(...)) probably isn't the most efficient way to do this.
            centre = (int(round(x + (w/2))), int(round(y + (h/2))))
            print("Centre detected: {}".format(centre))
            centres.append(centre)

            # Drawing centres and bounding boxes
            if publish_images_with_bbs:
                # Centre (marked in red):
                cv.circle(img, centre, 5, (0,0,255), -1)
                color = [int(c) for c in colors[classIDs[i]]]
                # Bounding box:
                cv.rectangle(img, (x, y), (x + w, y + h), color, 2)
                # Class and confidence label:
                text = "{}: {:.4f}".format(classes[classIDs[i]], confidences[i])
                cv.putText(img, text, (x, y - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    return [img, centres]

# TODO Temp - need to develop a custom message format
def gen_msg_string(image_time, image_dimensions, centres):
    msg_string = "{}.{}".format(image_time.secs, image_time.nsecs)
    msg_string += ";{}.{}".format(image_dimensions[0], image_dimensions[1])
    for centre in centres:
        msg_string += ";{}.{}".format(centre[0], centre[1])
    return msg_string

def on_image(image_msg):
    # process
    # publish
    # Get timestamp from picture and embed it into message to be sent.
    
    start_time = time.time()

    # Convert image to OpenCV format.
    # TODO Mutability - can we just do this overwriting in Python?
    image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
    # Run the detection
    [image, centres] = run_network(image)

    # Generate coordinate msg and publish
    # TODO Temporary solution - send as string [will make custom message type]
    # secs.nsecs;image_height.image_width;x1.y2;x2.y2;...
    # Get the time the image was taken.
    image_time = image_msg.header.stamp
    image_dimensions = (image_msg.height, image_msg.width)
    coord_pub.publish(gen_msg_string(image_time, image_dimensions, centres))

    # Publish image with bounding boxes
    if publish_images_with_bbs:
        image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
    
    # 'Forward propagation'
    print(f'Time to load, detect and publish: {(time.time() - start_time):.3}s')

    print("Requesting next image")
    # Publish an image request.
    request_pub.publish()

# Command line argument processing
if '-h' in sys.argv:
    # TODO Make sure these topic names are up to date.
    # TODO Should topic names start with '/'?
    print()
    print("*** Trophy Detector ***")
    print()
    print("Employs a YOLO v3 network to detect trophy instances.")
    print()
    print("2D coordinates of detected trophy centres on the camera1 image plane are published to the topic '/detected_trophy_centres'.")
    print()
    print("To also publish the images from the camera with trophy bounding boxes, start the detector with")
    print()
    # TODO Should this be 'python3'?
    print("   python3 trophy_detector.py -i")
    print()
    print("Output images with detected trophies labelled (with bounding boxes) will be published to the topic '/image_trophies_detected'.")
    print()
    exit(0)
elif  '-i' in sys.argv:
    # bbs = bounding boxes
    publish_images_with_bbs = True
    print()
    print("*** Publishing images with bounding boxes to topic '/image_trophies_detected'. ***")
    print()
else:
    # bbs = bounding boxes
    publish_images_with_bbs = False

setup_yolo()
rospy.init_node('trophy_detector')
bridge = CvBridge()

###### Image subscriber ######
# queue_size = 1 so that it only processes the most recent image received - stops lag
# (labelling of trophies in images which were actually received some time ago and no
# longer reflect the true position of the robot), but the image stream will be less continuous.
# Buffer size based on queue_size (1) x average message size
# TODO Still need to experiment with buffer size
image_sub = rospy.Subscriber("image_for_trophy_detection", Image, on_image)#, queue_size=1)

# TODO Change this description if the camera being used changes from camera1.
###### Publishers (trophy coords, images with bounding boxes, camera1 image requests) ######

# Publishes, for each image: (image timestamp, [(x1, y1), (x2, y2), ...])
# The timestamp is needed for tf, so that the change of coordinates from the 'camera1' frame into the 'map' frame is taken at the right point in time.
# [(x1, y1), (x2, y2), ...] is a list of trophy centres, one (x, y) 2-tuple for each trophy in the image.
# TODO [IMMEDIATE] Need to sort out message type for sending timestamped messages.
coord_pub = rospy.Publisher("detected_trophy_centres", String) # rospy.Publisher("detected_trophy_centres", Msg)
if publish_images_with_bbs:
    image_pub = rospy.Publisher("image_trophies_detected", Image)

# This node requests a new image for trophy detection once it has finish processing the current one (to prevent lag).
# The request is picked up by the node '', which forwards on an image.
# To request the next image for trophy detection:
request_pub = rospy.Publisher("trophy_image_request", Empty)

# TODO Temporary [need a more robust solution for this]!! Wait for the forwarder to start.
rospy.sleep(1)#3)
print("*** Ready ***")

# Request the first image:
request_pub.publish() #"Request")

while not rospy.is_shutdown():
    rospy.spin()