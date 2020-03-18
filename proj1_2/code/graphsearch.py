from heapq import heappush, heappop, heapify # Recommended.
import numpy as np
import time
from flightsim.world import World
from flightsim.axes3ds import Axes3Ds
import matplotlib.pyplot as plt
from proj1_2.code.occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):

    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    print('Start:', start_index)
    print('Goal:', goal_index)

    Cost_dic = {}
    Parent_dic = dict()

    ind = np.argwhere(occ_map.map == False)
    for _,idx in enumerate(ind):
        Cost_dic.update({tuple(idx):np.inf})
        Parent_dic.update({tuple(idx):(0,0,0)})

    Cost_dic[start_index] = 0
    visited = []
    children = []
    node = 0
    min_cost = [Cost_dic[start_index],start_index]  
    goal_flag = 0
    impossible = 0
    flag = True

    if goal_index in Cost_dic.keys():
        while goal_flag == 0:
            U = min_cost[1]
            neigh = get_neighbour(U)
            neigh_idx = 0
            d = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            while neigh and neigh_idx < len(neigh):
                if neigh[neigh_idx] in Cost_dic.keys():
                    if astar == True:
                        d[neigh_idx] = Cost_dic[U] + np.linalg.norm(np.asarray(goal_index)- np.asarray(neigh[neigh_idx])) # <--------------<< Cost to go in case of A*
                    else:
                        d[neigh_idx] = Cost_dic[U] + 1   #<------------------<< Cost to go 
                    if neigh[neigh_idx] == goal_index:
                        print('Reached Goal')
                        goal_flag = 1
                    


    return path


def get_neighbour(U):
    up1     = tuple(np.array([U[0]+1,U[1]+1,U[2]+1]))
    up2     = tuple(np.array([U[0]+1,U[1],U[2]+1]))
    up3     = tuple(np.array([U[0]+1,U[1]-1,U[2]+1]))
    up4     = tuple(np.array([U[0]-1,U[1]+1,U[2]+1]))
    up5     = tuple(np.array([U[0]-1,U[1],U[2]+1]))
    up6     = tuple(np.array([U[0]-1,U[1]-1,U[2]+1]))
    up7     = tuple(np.array([U[0],U[1]+1,U[2]+1]))
    up8     = tuple(np.array([U[0],U[1],U[2]+1]))
    up9     = tuple(np.array([U[0],U[1]-1,U[2]+1]))

    down1   = tuple(np.array([U[0]+1,U[1]+1,U[2]-1]))
    down2   = tuple(np.array([U[0]+1,U[1],U[2]-1]))
    down3   = tuple(np.array([U[0]+1,U[1]-1,U[2]-1]))
    down4   = tuple(np.array([U[0]-1,U[1]+1,U[2]-1]))
    down5   = tuple(np.array([U[0]-1,U[1],U[2]-1]))
    down6   = tuple(np.array([U[0]-1,U[1]-1,U[2]-1]))
    down7   = tuple(np.array([U[0],U[1]+1,U[2]-1]))
    down8   = tuple(np.array([U[0],U[1],U[2]-1]))
    down9   = tuple(np.array([U[0],U[1]-1,U[2]-1]))    

    
    left     = tuple(np.array([U[0]+1,U[1],U[2]]))
    leftd1   = tuple(np.array([U[0]+1,U[1]+1,U[2]]))
    leftd2   = tuple(np.array([U[0]+1,U[1]-1,U[2]]))
    right    = tuple(np.array([U[0]-1,U[1],U[2]]))
    rightd1  = tuple(np.array([U[0]-1,U[1]+1,U[2]]))
    rightd2  = tuple(np.array([U[0]-1,U[1]-1,U[2]]))
    front    = tuple(np.array([U[0],U[1]+1,U[2]]))
    back     = tuple(np.array([U[0],U[1]-1,U[2]]))

    neigh = [up1,up2,up3, up4,up5,up6,up7,up8,up9,down1,down2,down3,down4,down5,down6,down7,down8,down9,left,leftd1,leftd2,right,rightd1,rightd2,front,back]
    return neigh

