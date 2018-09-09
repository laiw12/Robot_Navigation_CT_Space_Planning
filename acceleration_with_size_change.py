

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




## parameters---------------------------------------------------------------------------------------------------------

## resolution
r_s = 0.5
# start point
start_point = [0,15,0]

## goal point
goal_point = [30,15,30]

safe_boundary = 0

## define initial velocity and acceleration  
v0 = 0
acceleration = 0.0008


# obstacle params --------------------------------------------------------------------------------------

## an accelreated obstacle
radius = 3
x_center = 30
y_center = 15
height = 30







def all_neighbours(point_array,obstacle):
    
    neighbour = []
    
    v = 0.5
    d = 0.5/2**(0.5)
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
        
    return obstacle_avoid(neighbour,obstacle) 


    




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
         







## obstacle avoid for moving cylinder with constant velociy and same direction 
def obstacle_avoid(valid_neighbours,obstacle):
    obstacle_free = []
    invalid_points = []
   
    for i in range(len(valid_neighbours)):
        for j in range(len(obstacle)):
            if valid_neighbours[i][2] == obstacle[j][2]:
               # if valid_neighbours[i] == [8,8,8]:
                   # print('yes')
                   # print(obstacle[j])
                
                if ellucian_distance(valid_neighbours[i],obstacle[j]) <= obstacle[j][3]+ safe_boundary:
                    if valid_neighbours[i] not in invalid_points:
                        invalid_points.append(valid_neighbours[i])
                    #    print(invalid_points)
    
    for i in range(len(valid_neighbours)):
        if valid_neighbours[i] not in invalid_points:
           # if valid_neighbours[i] == [8,8,8]:
              #  print('invalid_points')
               # print(invalid_points)
               obstacle_free.append(valid_neighbours[i])
        
    
    return obstacle_free
                
        
     
 



## the displacement at a given initial velocitya and acceleration
def acceleration_distance(t):
    t = t**2
    m = v0 + acceleration * 0.5 *(t**2)
    return m






           
   
##
def generate_cylinder_1(x_center,y_center,height):
    obstacle = []
    n = 0
    l= radius
    while n <= height:
          obstacle.append([x_center-acceleration_distance(n),y_center,n,l])
          n += r_s
          l += 0.1
   
    return obstacle



        
## change the string_point represenatation to int_list representation:

def to_int_list(final_path):
    
  
    for i in range(len(final_path)):
        final_path[i] =  final_path[i].split('_')
       
        for j in range(3):
            final_path[i][j] = float( final_path[i][j])
    
    return final_path
            
        

def ellucian_distance2(x,y):
    
    x1 = x[0]
    y1 = x[1]
   
    
    x2 = y[0]
    y2 = y[1]
   
    
    return ((x1-x2)**2 + (y1-y2)**2 )**(1/2)



 

## function that performs A* searh in a 3D graph. 
def A_Star(start,goal,obstacle):
  
    
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
        print(f_g_current)
        current = f_g_current[2]
        open_set.remove(f_g_current[2])
        
        
        if ellucian_distance2(current[0:2],goal[0:2])   <= 1:
       # if current == goal:
        
            print("path found, job finished")
          
            
            final_path =  reconstruct_path(came_from,current)
            
            return to_int_list(final_path)
           
          
        
        closed_set.append(current)
        
        neighbours = all_neighbours(current,obstacle)
        
        
        
        for i in range(len(neighbours)):
            if neighbours[i] in closed_set:
                continue
            if neighbours[i] in open_set:
                continue
            
            h_value = ellucian_distance2(neighbours[i][0:2],goal_point[0:2])
            g_score = ellucian_distance2(neighbours[i][0:2],current[0:2]) + f_g_current[1]
            f_score =  h_value
        
        
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

ob1 = generate_cylinder_1(30,15,30)

obt = ob1 

start_time = time.time()
test = A_Star(start_point,goal_point,obt)
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

ax.set_xlabel('x-position')
ax.set_ylabel('y-possition')
ax.set_zlabel('time')

# set axis label and  scale

ay.set_aspect('equal')
#ay.autoscale(tight=True)



ay.set_xlabel('x-position')
ay.set_ylabel('y-possition')

ay.set_xlim(0,30)
ay.set_ylim(0,30)



## initialize the start and goal point on the graph
ax.scatter(start_point[0],start_point[1],start_point[2],c ='red',s = 30)




if add_notation:
    ax.text(start_point[0]-5,start_point[1]-1,start_point[2]+3,'start')
    ax.text(goal_point[0]-1,goal_point[1]-1,goal_point[2]+3,'goal')
    
    ay.text(start_point[0],start_point[1]+3,'start')
    ay.text(goal_point[0]+1,goal_point[1]+1,'goal')



o = radius 

for i in range(len(a[0])):
    
    ax.scatter(a[0][i],a[1][i],a[2][i],c='black')
    ay.scatter(a[0][i],a[1][i],s = 10,c = 'black')
    l = radius
    if i <= 60:
   
        draw_circle_on_plane(x_center-acceleration_distance(i/2),y_center,o,i/2,ax)

        circle1 = plt.Circle((x_center-acceleration_distance(i/2),y_center), radius, color='r')
        ay.add_artist(circle1)
    o = o + 0.1
    
  





 
   # plt.pause(0.1)
    matplotrecorder.save_frame()
    if i <= 59:
        circle1.remove()
     





