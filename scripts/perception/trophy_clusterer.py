import sys, rospy
# TODO Temp - need a better message format.
from std_msgs.msg import String
#from farscope_robot_utils import where_is_this_trophy
from trophy_locater import where_is_this_trophy, where_are_these_trophies
import json
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
    #print(where_are_these_trophies(trophy_centres))
    #shelf_id, level_id,  = where_are_these_trophies(trophy_centres)
    #print('shelf id: ', shelf_id)
    #print('level id:', level_id)
    #print('position: ', pos_on_shelf)
    #print('coords: ',coords)

    #print(type(trophy_centres))
    
    for i ,centre in enumerate(trophy_centres):
        shelf_id, level_id, pos_on_shelf, coord = where_is_this_trophy(centre)
        #print('shelf id: ', shelf_id, 'level_id: ', level_id, 'position: ', pos_on_shelf, 'coords: ', coord)
        dict_template = {'shelf id' : shelf_id, 'level_id': level_id, 'position on shelf': pos_on_shelf, 'coordinates': coord}

        enum_dict = {i:dict_template}
        
        #print(test)
        dict_string = json_dict(enum_dict)
        
        i = 1 + i    
        print(dict_string)

    pub.publish(String(dict_string))

    

def json_dict(input_dict):
    dict_string= json.dumps(input_dict, separators = (',', ':'))
    return dict_string












rospy.init_node('trophy_clusterer')

rospy.Subscriber('trophy_coord_ests_3d', String, on_estimates)

pub = rospy.Publisher('trophy_update', String, queue_size = 3)










#rospy.Publisher(...)

while not rospy.is_shutdown():
    rospy.spin()