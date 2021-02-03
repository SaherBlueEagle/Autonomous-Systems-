import numpy as np
import queue
import time
import sys
import os
import collections
from matplotlib import cm
from PIL import Image
from numpy import*
from scipy import ndimage
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
i = -1 # are horizontal
j = 0  # are vertical
value = 0
matrix = np.zeros((400,1000))
matrix_outx = np.zeros((400,1000))


img = Image.open('Maze.png').convert('L')
img_inverted = ImageOps.invert(img)

np_img = np.array(img_inverted)
np_img[np_img == 0] = 1 #not converted from 0 to 1 its converted to 0 and 255
# so above line remake the 0`s to 1 which is free space
# and the line below turns 255 into 0 which is obstacles
np_img[np_img ==255] = 0
np_img[np_img ==254] = 0
matrix = np_img
matrix_outx = np_img
np.set_printoptions(threshold=np.inf, linewidth=np.inf)  # turn off summarization, line-wrapping
with open('output2D_Valid_Arry.txt', 'w') as f:
    f.write(np.array2string(np_img))               
obstacles = ""
explored = ""
def is_obstacle(Cell_X,Cell_Y):
    if Cell_X <= 999 and Cell_Y <= 399 :
        valu = matrix[Cell_Y,Cell_X]
        if valu == 0:
            return True
    return False
def is_wall_cell(Cell_X,Cell_Y):
    if Cell_X == 0 and Cell_Y > 0:
        return True
    if Cell_X > 0 and Cell_Y == 0:
        return True
    if Cell_X == 0 and Cell_Y == 0:
        return True
    return False
def is_explored_cell(Cell_X,Cell_Y):
    if "i=" + str(Cell_X) in explored and "j=" + str(Cell_Y) in explored:
        return True
    return False
def add_explored_cell(Cel_X,Cel_Y):
    global explored
    str1 = "i=" + str(Cel_X)+ ","
    str2 = "j=" + str(Cel_Y)+ ","
    exploredx = str1+str2
    explored = explored + "".join(exploredx)
    return



def implement_BFS(start_point=[],goal_point=[]):
    global matrix
    var_queue = queue.Queue()
    start_x = start_point[0]
    start_y = start_point[1]
    end_x = goal_point[0]
    end_y = goal_point[1]
    matrix[end_x,end_y] = 8 # <===== give a flag / Higlight the goal point (enha teba 8 fe west el 0s we el 1s)
    return bfs_helper(matrix, (start_x, start_y),(end_x,end_y))
  
    
 
for iter in range(1):
    start_x,start_y = 0,0 # 
def bfs_helper(grid, start,goalp):
    global matrix
    queue = collections.deque([[start]])
    goal = matrix[goalp[0],goalp[1]]
    height = len(matrix)
    width = len(matrix[0])
    print("Usinf BFS : " + str(width))
    print("Goal cell changed from 1 to value '8' for only Highlighting it => " + str(goal))
    seen = set([start])
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        if grid[y][x] == goal:
            return path
        for x2, y2 in ((x,y+1) , (x,y-1), (x-1,y),(x+1,y),(x-1,y-1),(x+1,y+1),(x+1,y-1),(x-1,y+1)):
            if 0 <= x2 < width and 0 <= y2 < height and is_wall_cell(x2,y2)==False and is_obstacle(x2,y2)==False and (x2, y2) not in seen:
                queue.append(path + [(x2, y2)])
                seen.add((x2, y2))

def implement_DFS(start_point=[],goal_point=[]):
    global matrix
    start_x = start_point[0]
    start_y = start_point[1]
    end_x = goal_point[0]
    end_y = goal_point[1]
    matrix[end_x,end_y] = 8 # <===== give a flag / Higlight the goal point (enha teba 8 fe west el 0s we el 1s)
    return dfs_helper(matrix, (start_x, start_y),(end_x,end_y))
  
def dfs_helper(grid, start,goalp):
    global matrix
    stack =  ([[start]])
    goal = matrix[goalp[0],goalp[1]]
    height = len(matrix)
    width = len(matrix[0])
    print("Using DFS : " + str(width))
    print("Goal cell changed from 1 to value '8' for only Highlighting it => " + str(goal))
    visited = set([start])
    path = [start]
    reached = False
    while stack:
        path = stack.pop(0)
        #print("here1:" + str(path))
        x, y = path[-1]
        #stack.append(path + [(x,y)])
        if grid[y][x] == goal or (y==goalp[0] and x ==goalp[1]):
            reached = True
            return path
        for x2, y2 in ((x,y-1) , (x-1,y-1), (x-1,y),(x-1,y+1),(x,y+1),(x+1,y+1),(x+1,y),(x+1,y-1)):
            if 0 <= x2 < width and 0 <= y2 < height and is_wall_cell(x2,y2)==False and is_obstacle(x2,y2)==False and (x2, y2) not in visited:
                #print("here2: "+ str(path))
                stack.append(path + [(x2,y2)])
                visited.add((x2, y2))
        
#============================ MAIN BFS CALL ==========================
#++++++++++++++++++++++++++++
#=====================================================================
#=====================================================================

BFSstart = time.time() #  <========= capture time before execution

my_start = [50, 40] # giving start position i,j    <=======================================    start point
my_goal = [250, 820] # giving end position j,i<=======================================    goal point

path_arry_list = implement_BFS(my_start,my_goal) #<=================================== needed BFS method takes start and goal points only
print(path_arry_list) 
length = len(path_arry_list)

print('============== Given Path Using BFS ==============')

for ipj in range(length):
    x_path = path_arry_list[ipj][0]
    y_path = path_arry_list[ipj][1]
    matrix_outx[y_path,x_path] = 0 
    print(" move in j : " +str(y_path) + " then " + "move in i : " +str(x_path) ) 
matrix_outx[matrix_outx == 4] = 1  
matrix_outx[matrix_outx >4] = 130
BFSend = time.time()#  <========= capture time after execution
print ("Goal Reached ! , Time taken to find goal : " + str(BFSend-BFSstart))
np.set_printoptions(threshold=np.inf, linewidth=np.inf)

im = Image.fromarray(matrix_outx*255)
im.save("Output_BFS.png")
#=========================================DFS==================================
with open('path_out_BFS.txt', 'w') as f:
    f.write(np.array2string(matrix_outx)) 
print ("Path Drawn from start to goal which cell value is '8'")
print ("Path saved to 'Output_BFS.png' in same Dir of .py file If On Windows")

#============================ DFS Main RESET =========================
#++++++++++++++++++++++++++++
#=====================================================================
#=====================================================================
img = Image.open('Maze.png').convert('L')
img_inverted = ImageOps.invert(img)
np_img = np.array(img_inverted)
np_img[np_img == 0] = 1 #not converted from 0 to 1 its converted to 0 and 255
# so above line remake the 0`s to 1 which is free space
# and the line below turns 255 into 0 which is obstacles
np_img[np_img ==255] = 0
np_img[np_img ==254] = 0
matrix = np_img
matrix_outx = np_img



DFSstart = time.time() #  <========= capture time before execution
path_arry_list = None
path_arry_list = implement_DFS(my_start,my_goal) #<=================================== needed DFS method takes start and goal points only

print(path_arry_list) 
length = len(path_arry_list)
print('============== Given Path Using DFS ==============')

for ipj in range(length):
    x_path = path_arry_list[ipj][0]
    y_path = path_arry_list[ipj][1]
    matrix_outx[y_path,x_path] = 0
    print(" move in j : " +str(y_path) + " then " + "move in i : " +str(x_path) ) 
matrix_outx[matrix_outx == 4] = 1  
matrix_outx[matrix_outx >4] = 130
DFSend = time.time()#  <========= capture time after execution
print ("Goal Reached ! , Time taken to find goal : " + str(DFSend-DFSstart))
np.set_printoptions(threshold=np.inf, linewidth=np.inf)
im = Image.fromarray(matrix_outx*255)
im.save("Output_DFS.png")

with open('path_out_DFS.txt', 'w') as f:
    f.write(np.array2string(matrix_outx)) 
print ("Path Drawn from start to goal which cell value is '8'")
print ("Path saved to 'Output_DFS.png' in same Dir of .py file If On Windows")
