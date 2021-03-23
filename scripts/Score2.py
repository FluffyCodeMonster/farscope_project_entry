# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 16:30:26 2021

@author: ep15603
"""

"""

Score 2 node 

"""
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def "function"

    rospy.loginfo("""""")

def Score2():
    
    rospy.init_node(‘Score2’, anonymous=True)
    
    pub = rospy.Publisher('movement', String, queue_size=2)
    pub2 = rospy.Publisher('stop', String, queue_size=2)
    
    sub = rospy.Subscriber('List', String, "function")
    sub2 = rospy.Subscriber('StandardInfo', String, "function")
    sub3 = rospy.Subscriber()
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        
        rospy.sleep()
        
        if __name__ == '__main__':
    try:
        Score2()
    except rospy.ROSInterruptException:
        pass