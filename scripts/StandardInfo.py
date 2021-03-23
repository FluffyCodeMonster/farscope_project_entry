# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 16:30:25 2021

@author: ep15603
"""


"""

Standard Info Node

"""
#!/usr/bin/env python

import rospy
import json



from std_msgs.msg import String
from strategy.srv import Info, InfoResponse

with open("scenario_file.json") as file:
    
    scenario = json.load(file)
    
with open("standardArm_file.json") as file:
    
    standard_arm = json.load(file)

with open("InfoGain_file.json") as file:
    
    info_gain = json.load(file)
    
def info_service(req):

    return InfoResponse(scenario, standard_arm, info_gain)

def standard_info():
    
    rospy.init_node(‘standard_info’, anonymous=True)
    s = rospy.Service('info', Info , info_service)
    rospy.spin()

 
        
if __name__ == '__main__':
    standard_info()
    