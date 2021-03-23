# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 17:30:38 2021

@author: ep15603
"""

"""

Score 1 node 

"""
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def "function"
    rospy.loginfo("""""")

def Score1():
    
    rospy.init_node(‘Score1’, anonymous=True)
    
    pub = rospy.Publisher('movement', String, queue_size=2)
    
    sub = rospy.Subscriber('List', String, "function")
    sub2 = rospy.Subscriber('StandardInfo', String, "function")
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        
        rospy.sleep()
        
        if __name__ == '__main__':
    try:
        Score1()
    except rospy.ROSInterruptException:
        pass