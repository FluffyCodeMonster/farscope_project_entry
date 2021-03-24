#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# from Strategy.srv import ListRequest, Info
import json

    
def calculate_score1(trophy_list, info):
    max_val = (0, None)
    for trophy in trophy_list:
        pos = trophy["pos"]
        deploy_time = info["deploy"][trophy]
        n_density = calculate_n_density(trophy, info)
        info_gain = info["info_gain"][trophy]
        score = 1.0 * deploy_time + 0 * n_density + 0 * info_gain
        if score > max_val[0]:
            max_val = (score, trophy)

    return max_val[1]


def calculate_n_density(trophy, info):
    return 0.5


def main():
    
    rospy.init_node("Score1", anonymous=True)
    
    # get_list = rospy.ServiceProxy('get_list', ListRequest)
    # get_info = rospy.ServiceProxy('info', Info)
    
    # goal_trophy = calculate_score1(get_list, get_info)
    # convert into message

    message = {
        "id": "131",
        "description": {
            "x": 0.0,
            "y": -2.5,
            "alpha": -1.5706,
            "z": 2
        }
    }
    
    pub = rospy.Publisher('/target_move', String, queue_size=2)
    msg = json.dumps(message)
    pub.publish(msg)
    

if __name__ == '__main__':
    main()
