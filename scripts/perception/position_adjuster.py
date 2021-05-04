#!/usr/bin/env python
# FT 04/05/21

# TODO Would this be better as a service?

import threading
import sys, rospy
# Use an empty message to send a signal.
from std_msgs.msg import Empty, Int32, String

# TODO Will these two block each other? Do I need multithreading? Could multithreading lead
# to a race condition s.t. a request is missed?

# TODO These parameters all need setting.
### GLOBALS ###
# Position (in pixels, from (x,y) midpoint) of left-hand edge of region trophy centre needs to be within to be successfully gripped.
grip_strip_min = 50
# Position (in pixels, from (x,y) midpoint) of left-hand edge of region trophy centre needs to be within to be successfully gripped.
grip_strip_max = 70

# Vertical constraints to look for the trophy between.
height_bound_max = 100
height_bound_min = 0
###############

class RequestListener (threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
    
    def run(self):
        print('Starting request listener thread')
        request_sub = rospy.Subscriber("position_check_request", Empty, on_request)

# TODO Also duplicated from trophy_projector.py. Create a module.
def between(num1, num2, test_num):
    # e.g. num1 = 5, num2 = 3.
    if (num1 >= num2):
        return (num2 <= test_num <= num1)
    else:   # if num1 < num2
        return (num1 <= test_num <= num2)

# TODO Note: code duplication from trophy_projector.py. Move this all into a module.
# Need to be especially careful of the protocol changing.
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

def on_centres(centres_msg):
    global position_check_requested
    # If a position check has been requested:
    if position_check_requested:
        position_check_requested = False
        
        # Deserialise image information.
        _, image_dims, centres = parse_centres_string(centres_msg.data)

        # Convert centres to relative coordinates:
        # CAUTION: image_dims = (height, width) - not the way round you'd expect!
        image_height = image_dims[0]
        image_width = image_dims[1]
        image_centrepoint = (float(image_width) / 2, float(image_height) / 2)

        possible_trophies = []

        # Only one centre should be in the testing location. Need to identify it.
        # Vertical constraints are hardcoded.
        # For the horizontal, the trophy closest to the centre of the gripping region is selected as the one to test the position of.
        for centre in centres:
            # Define +ve to right, +ve upwards.
            # (0,0) is at the top-left of the images.
            x_disp = centre[0] - image_centrepoint[0]
            y_disp = image_centrepoint[1] - centre[1]

            if between(height_bound_min, height_bound_max, y_disp):
                # Add trophy to set of possible trophies
                possible_trophies.append(x_disp)
        
        # If it can't find any trophies between the min and max heights:
        if isempty(possible_trophies):
            image_pub.publish(-2)

        # Get the trophy closest to the middle of the region.
        trophy_locations_relative = []
        for trophy in possible_trophies:
            trophy_locations_relative.append(abs(x_disp - grip_strip_midpoint))
        
        # Position of the closest trophy in the image.
        closest_trophy_position = possible_trophies[trophy_locations_relative.index(min(trophy_locations_relative))]

        # If it is seen in the region: 0; if it is seen to the right: +1; if it is seen to the left: -1; if it is not seen: -2.
        # Note: -2 does not mean that the trophy *is* to the left, it just means that it can't be
        # seen to the right. This distinction is made because the arm might be occluding part of the view.
        # There is also the possibility that the trophy is to the left and occluded, and so the trophy taken to be closest
        # is one further away but in view to the right.
        #
        # If in the grabbing region:
        if between (grip_strip_min, grip_strip_max, closest_trophy_position):
            image_pub.publish(0)
        elif closest_trophy_position > grip_strip_max):
            image_pub.publish(1)
        else:
            image_pub.publish(-1)
        
        print('Adjustment response published')

def on_request(request):
    global position_check_requested
    print("Position check request received")
    position_check_requested = True

grip_strip_midpoint = (grip_strip_min + grip_strip_max) / 2

position_check_requested = False

rospy.init_node('position_adjuster')
image_sub = rospy.Subscriber("detected_trophy_centres", String, on_image, queue_size=1)
image_pub = rospy.Publisher("final_adjustment", Int32)

# Start request listener thread:
request_listener = RequestListener()
request_listener.start()

# Keep alive
while not rospy.is_shutdown():
    rospy.spin()