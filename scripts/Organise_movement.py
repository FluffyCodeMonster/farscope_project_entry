# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 16:43:20 2021

@author: ep15603
"""

"""

Organise movement node 

"""
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def "function"
    rospy.loginfo("""""")

def OrganiseMovement():
    
    rospy.init_node(‘OrganiseMovement’, anonymous=True)
    
    pub = rospy.Publisher('moveBaseArm', String, queue_size=2)

    sub = rospy.Subscriber('MovementDone', String, "function")
    
"can base_moved and arm_moved publish on sam topic moveBaseArm?"
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        
        rospy.sleep()
        
        if __name__ == '__main__':
    try:
        OrganiseMovement()
        
    except rospy.ROSInterruptException:
        pass