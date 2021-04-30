#!/usr/bin/env python
import sys, rospy
# TODO Temp - need a better message format.
from std_msgs.msg import String
#from farscope_robot_utils import where_is_this_trophy
from trophy_locater import where_is_this_trophy, where_are_these_trophies

# Coming from 'trophy_projector.py'
def parse_input(msg_string):
    # String format:
    # secs.nsecs;x1:y1:z1;x2:y2:z2;...
    # x, y, z are floats - cannot use '.' to delimit them - have to use ':' instead.
    centres = []
    msg_split = msg_string.split(';')

    # Parsing time.
    dot_split = msg_split[0].split('.')
    time = rospy.Time(int(dot_split[0]), int(dot_split[1]))

    # Parsing centre coords.
    for centre_string in msg_split[1:]:
        # TODO Should this be double bracketed?
        dot_split = centre_string.split(':')
        centres.append((float(dot_split[0]), float(dot_split[1]), float(dot_split[2])))
    
    return [time, centres]

def on_estimates(centre_estimates):
    [time, trophy_centres] = parse_input(centre_estimates.data)



    # Testing
    #print()
    #print(" *** New centres *** ")
    #print("Time: {}s, {}nsec".format(time.secs, time.nsecs))
    print(where_are_these_trophies(trophy_centres))

    #for centre in trophy_centres:
       # print(centre)
        #print(where_are_these_trophies(centre))





rospy.init_node('trophy_clusterer')

rospy.Subscriber('trophy_coord_ests_3d', String, on_estimates)

#rospy.Publisher(...)

while not rospy.is_shutdown():
    rospy.spin()