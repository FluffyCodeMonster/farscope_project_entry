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
from Strategy.srv import ListRequest, Info

class Score1:
    
    def __init__ (self):
        pass
    
    
    def calculate_score1(slef, deploy_time, n_density, info_desnity):
        
        score_trophy = []
        
        for trophy in trophy_list
        
            score = alpha1*info_gain + alpha2*deploy_time + alpha3*n_density
            score_trophy.append((score, trophy))
            
        return score_trophy = []
    
    def calculate_info_density(self, get_info):
        
        # use service get_info to calculate info_density
        
        return info_density
    
    def calculate_deploy_time(self, get_info):
        
        # Use standar path file to calculate deply time
        
        return deploy_time
        
    def calculate_n_density(self, get_list):
        
        #use trophy list to calculate n_density
        
        return n_density

def main():
    
    rospy.init_node(‘Score1’, anonymous=True)
    
    get_list = rospy.ServiceProxy('get_list', ListRequest )
    get_info = rospy.ServiceProxy('info', Info )
    
    n_density = calculate_n_density(get_list)
    deploy_time = calculate_deploy_time(get_info)
    info_desnity = calculate_info_density(get_info)
    
    score_trophy = calculate_score1(n_density, deploy_time, info_density)
    
    #Find trophy with maximum score
    
    max_score = numpy.where(score_trophy == numpy.amax(score_trophy))
    
    pub = rospy.Publisher('\movement', String, queue_size=2)
    pub.publish(max_score)
    rate = rospy.Rate(10)
    

        
if __name__ == '__main__':
    main()