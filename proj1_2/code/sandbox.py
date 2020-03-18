import inspect
import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import time

from flightsim.axes3ds import Axes3Ds
from flightsim.world import World
from proj1_2.code.occupancy_map import OccupancyMap
from proj1_2.code.graph_search import graph_search
from proj1_2.code.Astar import Astar
# from proj1_2.code.newdijk import newdijk

# Choose a test example file. You should write your own example files too!
# filename = 'test_empty.json'
# filename = 'test_saw.json'
# filename = 'test_forest.json'
filename = 'test_window.json'
# filename = 'test_impossible.json'

# Load the test example.
file = Path(inspect.getsourcefile(lambda:0)).parent.resolve() / '..' / 'util' / filename
world = World.from_file(file)          # World boundary and obstacles.
resolution = world.world['resolution'] # (x,y,z) resolution of discretization, shape=(3,).
margin = world.world['margin']         # Scalar spherical robot size or safety margin.
start  = world.world['start']          # Start point, shape=(3,)
goal   = world.world['goal']           # Goal point, shape=(3,)

# Run your code and return the path.
start_time = time.time()
path = graph_search(world, resolution, margin, start, goal, astar=True)
# path = newdijk(world, resolution, margin, start, goal, astar=False)
# path = Astar(world, resolution, margin, start, goal, astar=False)
end_time = time.time()
print(f'Solved in {end_time-start_time:.2f} seconds')

# Draw the world, start, and goal.
fig = plt.figure()
ax = Axes3Ds(fig)
world.draw(ax)
ax.plot([start[0]], [start[1]], [start[2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='none')
ax.plot( [goal[0]],  [goal[1]],  [goal[2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='none')
# for i,idx in enumerate(costs):
#     if idx[0]< 10:
#         ax.plot( [costs[i][1][0]],  [costs[i][1][1]],  [costs[i][1][2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='blue')
#     elif idx[0]< 20:
#         ax.plot( [costs[i][1][0]],  [costs[i][1][1]],  [costs[i][1][2]], 'bo', markersize=10, markeredgewidth=3, markerfacecolor='green')
#     elif idx[0]< 30:
#         ax.plot( [costs[i][1][0]],  [costs[i][1][1]],  [costs[i][1][2]], 'yo', markersize=10, markeredgewidth=3, markerfacecolor='yellow')
#     elif idx[0]< 40:
#         ax.plot( [costs[i][1][0]],  [costs[i][1][1]],  [costs[i][1][2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='red')

    

# Plot your path on the true World.
if path is not None:
    world.draw_line(ax, path, color='blue')
    world.draw_points(ax, path, color='blue')

# For debugging, you can visualize the path on the provided occupancy map.
# Very, very slow! Comment this out if you're not using it.
# fig = plt.figure()
# ax = Axes3Ds(fig)
# oc = OccupancyMap(world, resolution, margin)
# oc.draw(ax)
# ax.plot([start[0]], [start[1]], [start[2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='none')
# ax.plot( [goal[0]],  [goal[1]],  [goal[2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='none')
# if path is not None:
#     world.draw_line(ax, path, color='blue')
#     world.draw_points(ax, path, color='blue')

plt.show()
