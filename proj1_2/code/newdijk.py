from heapq import heappush, heappop, heapify # Recommended.
import numpy as np
import time
from flightsim.world import World
from proj1_2.code.occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    print('Start:', start_index)
    print('Goal:', goal_index)

    Cost_dic = {}
    Parent_dic = dict()
    # start_time = time.time()
    for i,_ in enumerate(occ_map.map[:]):
        for j,_ in enumerate(occ_map.map[i][:]):
            for k,_ in enumerate(occ_map.map[i][j][:]):
                if occ_map.map[i][j][k] == False:   # Only consider points where there are no obstacles
                    parent = {tuple(np.array([i,j,k])):tuple(np.array([0,0,0]))}
                    Parent_dic.update(parent)   # Dictionary of Parents
                    if tuple(np.array([i,j,k])) != start_index:
                        cost = {tuple(np.array([i,j,k])): np.inf}
                    else:
                        cost = {tuple(np.array([i,j,k])): 0}
                    Cost_dic.update(cost)       # Dictionary of costs
    # end_time1 = time.time()
    # print(f'Mapping in {end_time1-start_time:.2f} seconds')
    heap = []
    pathq = []

    # for keys in Cost_dic.keys():
    #     heappush(heap,[Cost_dic[keys],keys])                 # Priority Q

    node = 0
    
    heappush(heap,[Cost_dic[start_index],start_index])
    min_cost = heappop(heap)  #<---------------<<<<< Reuse this
    
    # ---------- Done with intialization ------------------
    start_time = time.time()
    goal_flag = 0
    impossible = 0
    flag = True
    # while heap and min_cost[0] < np.inf:
    

    if goal_index in Cost_dic.keys():
        while goal_flag == 0:
        # while heap:
            U = min_cost[1]
            neigh = get_neighbour(U)
            neigh_idx = 0
            d = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # <-------<< List to store cost of all neigbours
            while neigh and neigh_idx < len(neigh):
                if neigh[neigh_idx] in Cost_dic.keys():                
                    if astar == True:
                        d[neigh_idx] = Cost_dic[U] + np.linalg.norm(np.asarray(goal_index)- np.asarray(neigh[neigh_idx])) # <--------------<< Cost to go in case of A*
                    else:
                        d[neigh_idx] = Cost_dic[U] + 1   #<------------------<< Cost to go 
                    if neigh[neigh_idx] == goal_index:
                        print('Reached Goal')
                        goal_flag = 1
                    if d[neigh_idx] < Cost_dic[neigh[neigh_idx]]:   # <---------------<< If new cost to go is smaller, update the value                   
                        # heap[heap.index([Cost_dic[neigh[neigh_idx]],neigh[neigh_idx]])] = [d[neigh_idx],neigh[neigh_idx]]

                        Cost_dic[neigh[neigh_idx]] = d[neigh_idx]
                        heappush(heap,[Cost_dic[neigh[neigh_idx]],neigh[neigh_idx]])
                        Parent_dic[neigh[neigh_idx]] = U

                neigh_idx += 1  # <-------------<< Goto next Neighbour
            # heapify(heap)

            if impossible != 1 and heap ==[]:
                impossible = 1
                flag = False
                break

            min_cost = heappop(heap)  #<---------------<<<<< Reuse this
            node += 1

        # print('Heap is empty') 

        print('Nodes: ', node)
        path = []   
        point = goal_index

        # start_time = time.time()
        while flag is True:
            mtpoint = occ_map.index_to_metric_center(Parent_dic[point])
            # mtpoint = occ_map.index_to_metric_negative_corner(Parent_dic[point])
            path.append(mtpoint)
            point = Parent_dic[point]
            if point == start_index:
                flag = False
        if impossible == 0:      
            path.append(start)  
            path.reverse()
            path.append(goal)
            path = np.asarray(path)
            length = float(round(np.sum(np.linalg.norm(np.diff(path, axis=0),axis=1)),3))
            print('Path Length: ', length)
        elif impossible == 1:
            path = None
        return path

    else:
        goal_flag == 1
        path = []
        path = [start_index]
        path = np.asarray(path)
        print('Goal Not Reachable')

        return  None


# def get_neighbour(U):
#     up     = tuple(np.array([U[0],U[1],U[2]+1]))
#     down   = tuple(np.array([U[0],U[1],U[2]-1]))
#     left   = tuple(np.array([U[0]+1,U[1],U[2]]))
#     right  = tuple(np.array([U[0]-1,U[1],U[2]]))
#     front  = tuple(np.array([U[0],U[1]+1,U[2]]))
#     back   = tuple(np.array([U[0],U[1]-1,U[2]]))

#     neigh = [up,down,left,right,front,back]
#     return neigh

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




# def neighbours(point_idx):
#     neigh_idx = np.zeros((3,3,3))
    

