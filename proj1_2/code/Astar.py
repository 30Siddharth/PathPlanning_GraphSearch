from heapq import heappush, heappop, heapify, heappushpop # Recommended.
import numpy as np

from flightsim.world import World
from proj1_2.code.occupancy_map import OccupancyMap # Recommended.

def Astar(world, resolution, margin, start, goal, astar):
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

    # Cost_dic = {}
    # Parent_dic = dict()

    # ind = np.argwhere(occ_map.map == False)
    # for _,idx in enumerate(ind):
    #     Cost_dic.update({tuple(idx):np.inf})
    #     Parent_dic.update({tuple(idx):(0,0,0)})

    # Cost_dic[start_index] = 0
    # print(Parent_dic[(5,15,4)])
    # heap = []
    
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
    visited = []
    children = []
    node = 0
    min_cost = [Cost_dic[start_index],start_index]  
    goal_flag = 0
    # ---------- Done with intialization ------------------

    while goal_flag == 0:
        U = min_cost[1]
        print(U)
        neighood = get_neighbour(U)
        neigh_idx = 0
        d = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # <-------<< List to store cost of all neigbours
        for neigh_idx,neighbr in enumerate(neighood):
            if neighbr in Cost_dic.keys():
                print(neighbr,neigh_idx)
                if d[neigh_idx] < Cost_dic[neighbr]:
                    d[neigh_idx] = Cost_dic[U] + 1
                    heappush(visited,[d[neigh_idx],neighbr])
                    # Cost_dic[neighbr] = d[neigh_idx]
                if neighbr == goal_index:
                    goal_flag = 1
                    print('Reached Goal!') 

        min_cost = min(d)
        min_pos = neighood.index(d.index())
        heappush(children,[min_pos,min(d)])
        node += 1
        min_cost = children[min_pos]
        Cost_dic[min_cost[1]] = min_cost[0]
        Parent_dic[min_cost[1]] = U
        # min_cost = heappop(heap)  #<---------------<<<<< Reuse this
        

    # print('Heap is empty') 
    # print('Goal Parent', Parent_dic[goal_index])
    print('Nodes: ', node)
    path = []   
    # cost_list = [] 
    flag = True
    point = goal_index

    # start_time = time.time()
    while flag is True:
        mtpoint = occ_map.index_to_metric_center(Parent_dic[point])
        # mtpoint = occ_map.index_to_metric_negative_corner(Parent_dic[point])
        path.append(mtpoint)
        point = Parent_dic[point]
        if point == start_index:
            flag = False
    path.append(start)  
    path.reverse()
    path.append(goal)
    path = np.asarray(path)

    # print(cost_list[1][1][1])
    return path


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


