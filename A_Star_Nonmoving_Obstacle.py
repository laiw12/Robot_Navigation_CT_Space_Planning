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
height = 30
elevation = -5
resolution = 100
color = 'r'
x_center = 15
y_center = 15






def all_neighbours(point_array):
    
    neighbour = []
    
    v= 1
    d = 0.71
    x = point_array[0]
    y = point_array[1] 
    z = point_array[2]
    
    
    # Vertical and Horizontal Moves 
    neighbour1 = [  [x+v,y,z+r_s],[x-v,y,z+r_s], [x,y+v,z + r_s], [x,y-v,z + r_s]]               
    
    # Diagonal Moves
    neighbour2 = [[x+d,y+d,z+r_s], [x-d,y-d,z+r_s], [x-d,y+d,z+r_s], [x+d,y-d,z+r_s]]
    
    
  #  neighbour5 = [ [x,y,z-r_s], [x+v,y,z-r_s],[x-v,y,z-r_s], [x,y+v,z - r_s], [x,y-v,z -r_s]]  
    
  # neighbour4 =  [[x+d,y+d,z-r_s], [x-d,y-d,z-r_s], [x-d,y+d,z-r_s], [x+d,y-d,z-r_s]]
    
 #   neighbour6 = [  [x+v,y,z],[x-v,y,z], [x,y+v,z ], [x,y-v,z]]  
    
  #  neighbour7 =   [[x+d,y+d,z], [x-d,y-d,z], [x-d,y+d,z], [x+d,y-d,z]]
    
    
    
    neighbour3 = neighbour1 + neighbour2 
    
    for i in range(len(neighbour3)):
        count = 0
        for j in range(3):
         
            if neighbour3[i][j] >= 0:
                count += 1
        if count == 3:
            neighbour.append(neighbour3[i])
        
       
                
                
        
    
    
   
    if avoid_obstacle:
        return obstacle_avoid(neighbour)
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
    

def ellucian_distance2(x,y):
    
    x1 = x[0]
    y1 = x[1]
   
    
    x2 = y[0]
    y2 = y[1]
   
    
    return ((x1-x2)**2 + (y1-y2)**2 )**(1/2)




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
       
        print(f_g_current)
        if ellucian_distance2(current[0:2],goal[0:2])   <= 2:
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
            
            h_value = ellucian_distance2(neighbours[i][0:2],goal_point[0:2])
            g_score = ellucian_distance2(neighbours[i][0:2],current[0:2]) + f_g_current[1]
            f_score = g_score + h_value
        
        
            openset_queue.put((f_score,g_score,neighbours[i]))
            open_set.append(neighbours[i])
            came_from[to_string(neighbours[i])] = to_string(current)
        
        
    
   
# function that plots a 3d cylinder  
def plot_3D_cylinder(plot,radius, height, elevation=0, resolution=100, color='r', x_center = 0, y_center = 0):
   

    x = np.linspace(x_center-radius, x_center+radius, resolution)
    z = np.linspace(elevation, elevation+height, resolution)
    X, Z = np.meshgrid(x, z)

    Y = np.sqrt(radius**2 - (X - x_center)**2) + y_center # Pythagorean theorem

    plot.plot_surface(X, Y, Z, linewidth=0, color=color)
    plot.plot_surface(X, (2*y_center-Y), Z, linewidth=0, color=color)

    floor = Circle((x_center, y_center), radius, color=color)
    plot.add_patch(floor)
    art3d.pathpatch_2d_to_3d(floor, z=elevation, zdir="z")

    ceiling = Circle((x_center, y_center), radius, color=color)
    plot.add_patch(ceiling)
    art3d.pathpatch_2d_to_3d(ceiling, z=elevation+height, zdir="z")

    plot.set_xlabel('x-position')
    plot.set_ylabel('y-possition')
    plot.set_zlabel('time')


    
    

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
print("--- %s seconds ---" % (time.time() - start_time))

test.reverse()

a = x_y_z_value(test)
        
        
y = [1,2,3,4,5,6,7]
x = [2,4,8,16,32,64,128]



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



circle1 = plt.Circle((x_center, y_center), radius, color='r')
ay.add_artist(circle1)





## initialize the start and goal point on the graph
ax.scatter(start_point[0],start_point[1],start_point[2],c ='red',s = 30)




if add_notation:
    ax.text(start_point[0]-5,start_point[1]-1,start_point[2]+3,'start')
    ax.text(goal_point[0]-1,goal_point[1]-1,goal_point[2]+3,'goal')
    
    ay.text(start_point[0],start_point[1]+3,'start')
    ay.text(goal_point[0]+1,goal_point[1]+1,'goal')



plot_3D_cylinder(ax,radius, height, elevation=elevation, resolution=resolution, color=color, x_center=x_center, y_center=y_center)
## generage frame using animation 
for i in range(len(a[0])):
    ax.scatter(a[0][i],a[1][i],a[2][i],c='black')
    ay.scatter(a[0][i],a[1][i],s = 10,c = 'black')
    
    matplotrecorder.save_frame()

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


