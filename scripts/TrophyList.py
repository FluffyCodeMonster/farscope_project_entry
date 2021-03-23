# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 16:33:30 2021

@author: ep15603
"""

"""
Trophy list node 

"""
#!/usr/bin/env python

import rospy
import yaml
import world
import json

from Strategy.srv import ListRequest, ListResponse  "==> format of .srv file ==> string name #name of commmand e.g. give me list
                                                    "                            ------------ 
                                                    "                            string name #return list, 
                                                    "                                         this probably won't be a string?
from std_msgs.msg import String

with open("scenario_file.yaml") as file:
    
    scenario = yaml.load(file)
    
with open("environment file.world") as file:
    
    environment = world.load(file)
    
class TropyList:
    def __init__(self):
        self.location_trophies = "info from scenario file"
        
        
    def get_list(self, req):
        rospy.loginfo('List requested')
         #Create list
         #Info from scenario file is already a list
         
        
        response = ListResponse()
        return response
    
    def perception_callback(self, msg):
        
        self.x_trophy_updated = "location of perception info in .msg repo"
        self.y_trophy_updated = "location of perception info in .msg repo"
        
        "Here the info for trophies are taken from .msg
        "provided by perception"

def main():
    
    rospy.init_node(‘create_list’, anonymous=True)
    list = TrophyList() 
    s = rospy.Service('get_list', ListRequest, list.get_list)
    
    sub = rospy.Subscriber('\perception', 3DMap, list.perception_callback)
    
    pub = rospy.Publisher('\list_organise', String, queue_size=2)
    rospy.spin()
    
        
if __name__ == '__main__':
    main()