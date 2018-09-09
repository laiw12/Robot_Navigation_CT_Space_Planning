# -*- coding: utf-8 -*-
"""
Created on Sat Mar  3 17:19:12 2018

@author: laiw1
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Feb 28 17:14:46 2018

@author: laiw1
"""

# -*- coding: utf-8 -

import time
from matplotrecorder import matplotrecorder
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from queue import PriorityQueue
import numpy as np
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d



## Debug and Visualization -----------------------------------------------------------------------------------
add_notation = True



##  Animation Parameter ----------------------------------------------------------------------------------
iframe = 0
donothing = False  # switch to stop all recordering

avoid_obstacle = True


## parameters---------------------------------------------------------------------------------------------------------

## resolution
r_s = 0.5
# start point
start_point = [0,0,0]

## goal point
goal_point = [30,30,30]



# obstacle params --------------------------------------------------------------------------------------
radius = 5
x_center = 10
y_center = 0
height = 30





## function that returns all neighbors of a point:
## takes single point
def all_neighbours(point_array):
    neighbour = []
    
    x = point_array[0]
    y = point_array[1] 
    z = point_array[2]
    
    
    x1 = x + r_s
    x2 = x - r_s
    x_cordinates =  [x,x1,x2]
    x_final_value = [x for x in x_cordinates if x >=0]
    
    y1 = y + r_s
    y2 = y - r_s
    y_cordinates =  [y,y1,y2]
    y_final_value = [y for y in y_cordinates if y >=0]
    
    z1 = z + r_s
    z2 = z - r_s
    z_cordinates =  [z,z1,z2]
    z_final_value = [z for z in z_cordinates if z >=0]
    
    for i in range(len(x_final_value)):
        for j in range(len(y_final_value)):
            for k in range(len(z_final_value)):
                neighbour.append([x_final_value[i],y_final_value[j],z_final_value[k]])
    
    #remove the orginal point
    
    neighbour.remove([x,y,z])
   
    if avoid_obstacle:
        return obstacle_avoid(neighbour,radius,height)
    else:
        return neighbour
    



## calculate the shortest distance between two points
## h function in A star search
def ellucian_distance(x,y):
    
    x1 = x[0]
    y1 = x[1]
    z1 = x[2]
    
    x2 = y[0]
    y2 = y[1]
    z2 = y[2]
    
    return ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**(1/2)
    


## Calculate f score for each point (using the h function)
def calculate_fscore(neighbour):
    for i in range(len(neighbour)):
        neighbour[i].append(ellucian_distance(neighbour[i],goal_point))
        
    return neighbour



## convert a list point to string in order to do hashing.
def to_string(x):
    string =''
    for i in range(len(x)):
        string +=  str(x[i]) + '_'
    
    string = string[0:-1]
    
    return string
        

## back_tracking the hash table to get the final path.        
def reconstruct_path(cameFrom,current):
    final_path = [to_string(current)]
    current = to_string(current)
    
    while current in cameFrom:
        current = cameFrom[current]
        final_path.append(current)
    
    return final_path
         



## obstacle elimination function ------ non_moving_cylinder
'''
def obstacle_avoid(valid_neighbours):
    
    above_height = []
    below_height = []
    valid_point = []
 
    for i in range(len(valid_neighbours)):
        if valid_neighbours[i][2] <= height:
            below_height.append(valid_neighbours[i])
        if valid_neighbours[i][2] > height:
            above_height.append(valid_neighbours[i])
        

 
    for i in range(len(below_height)):
        if ellucian_distance(below_height[i], [x_center,y_center,below_height[i][2]]) >= radius:
            valid_point.append(below_height[i])
    
    return valid_point + above_height
'''




## obstacle avoid for moving cylinder with constant velociy and same direction 
def obstacle_avoid(valid_neighbours,radius,height):
    obstacle = []
    obstacle_free = []
    
    i = 0
    while i <= height:
        obstacle.append([x_center,y_center+i,i])
        i += r_s
    
    for i in range(len(valid_neighbours)):
        for j in range(len(obstacle)):
            
            if valid_neighbours[i][2] == obstacle[j][2]:
                
  
                if ellucian_distance(valid_neighbours[i],obstacle[j]) >= radius:
                
                    obstacle_free.append(valid_neighbours[i])
             
    
    return obstacle_free
                
                
        
    



    
    
            
    
          
        
        
## change the string_point represenatation to int_list representation:

def to_int_list(final_path):
    
  
    for i in range(len(final_path)):
        final_path[i] =  final_path[i].split('_')
       
        for j in range(3):
            final_path[i][j] = float( final_path[i][j])
    
    return final_path
            
        

## function that performs A* searh in a 3D graph. 
def A_Star(start,goal):
  
    
    closed_set = []
    
    open_set = []
    
    ## used to get the lowest f score
    openset_queue = PriorityQueue()
    
    ## dictionary to do back tracking
    came_from = {}
    ## f, g
    openset_queue.put((0,0,start))
    open_set.append(start)
    
    
    while (openset_queue.empty() == False):
        
        f_g_current = openset_queue.get()
      
        current = f_g_current[2]
        open_set.remove(f_g_current[2])
       
        
        if current == goal:
            print("path found, job finished")
          
            
            final_path =  reconstruct_path(came_from,current)
            
            return to_int_list(final_path)
           
          
        
        closed_set.append(current)
        
        neighbours = all_neighbours(current)
        
        
        
        for i in range(len(neighbours)):
            if neighbours[i] in closed_set:
                continue
            if neighbours[i] in open_set:
                continue
            
            h_value = ellucian_distance(neighbours[i],goal_point)
            g_score = ellucian_distance(neighbours[i],current) + f_g_current[1]
            f_score = g_score + h_value
        
        
            openset_queue.put((f_score,g_score,neighbours[i]))
            open_set.append(neighbours[i])
            came_from[to_string(neighbours[i])] = to_string(current)
        
        
    
## function that plot a circle plane on a given height
def draw_circle_on_plane(x,y,r,height,graph):
    theta = np.linspace(0, 2*np.pi, 100)
    # compute x1 and x2
    x1 = r*np.cos(theta) + x
    x2 = r*np.sin(theta) + y

    graph.plot_wireframe(x1,x2,height,color = 'red')
    
    

def x_y_z_value(final_path):
    x = []
    y = []
    z = []
    for i in range(len(final_path)):
        x.append(final_path[i][0])
        y.append(final_path[i][1])
        z.append(final_path[i][2])
    
    return [x,y,z]







#--------------------------------------------------------------------------------------------------------------

start_time = time.time()
test = A_Star(start_point,goal_point)
print(test)
print("the length of the path is: " + str(len(test)))
print("--- %s seconds ---" % (time.time() - start_time))

test.reverse()

a = x_y_z_value(test)

   
        




fig = plt.figure()
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(121,projection = '3d')
ay = fig.add_subplot(133)

# Configuration_time_space_graph (3D)


## set x axis scale for the 3 _d grapph

ax.set_xlim3d(0,30)
ax.set_ylim3d(0,30)
ax.set_zlim3d(0,30)



# set axis label and  scale

ay.set_aspect('equal')
#ay.autoscale(tight=True)



ay.set_xlabel('x-position')
ay.set_ylabel('y-possition')

ay.set_xlim(0,30)
ay.set_ylim(0,30)








## initialize the start and goal point on the graph
ax.scatter(start_point[0],start_point[1],start_point[2],c ='red',s = 30)


ax.scatter(goal_point[0],goal_point[1],goal_point[2],c ='blue',s=30)


if add_notation:
    ax.text(start_point[0]-5,start_point[1]-1,start_point[2]+3,'start')
    ax.text(goal_point[0]-1,goal_point[1]-1,goal_point[2]+3,'goal')
    
    ay.text(start_point[0],start_point[1]+5,'start')
    ay.text(goal_point[0]+1,goal_point[1]+1,'goal')



# plot_3D_cylinder(ax,radius, height, elevation=elevation, resolution=resolution, color=color, x_center=x_center, y_center=y_center)
## generage frame using animation 
for i in range(len(a[0])):
    
    ax.scatter(a[0][i],a[1][i],a[2][i],c='black')
    ay.scatter(a[0][i],a[1][i],s = 10,c = 'black')
    if i <= 60:
   
        draw_circle_on_plane(10,i/2,radius,i/2,ax)
        circle1 = plt.Circle((x_center, 0+i/2), radius, color='r')
        ay.add_artist(circle1)

        
 
        

 
    plt.pause(0.1)
    matplotrecorder.save_frame()
    if i <= 59:
        circle1.remove()

"""
for angle in range(0, 360):
    ax.view_init(330, angle)
    plt.draw()
    plt.pause(.001)
"""
#plt.figure(1)
#plt.subplot(211)
#plt.plot(a[0],a[1])


#----------------------------------------------------------------------------------------------------










































    



## 2_D_ploting
#ax.plot_wireframe(a[0],a[1],0)
#ax.plot_wireframe(6,x,y)

#plt.show()

