# -*- coding: utf-8 -*-
"""
Created on Mon Mar 22 12:15:16 2021

@author: ep15603
"""

import yaml

with open("typical.yaml", 'r') as file:
    
    yaml_file = yaml.load(file)
    
    
full_shelves = [0]*8
trophy_list = []


for i in range(0,8):


    full_shelves[i] = yaml_file['scenario'][i]['trophies']
       
            

#print(full_shelves)

for i in range (0,8):
    for j in range(0,4):
       
       shelf = i
       trophy = full_shelves[i][j]
       trophy_list.append((shelf, trophy))
       #print(full_shelves[i][j])
       print(trophy_list)        
      
# Currently trophy list is giving correct output, as in it associates each trophy to its shelf ID
#However, it stops indicating IndexError: list index out of range
       