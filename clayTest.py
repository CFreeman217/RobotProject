import pathfinder

unknown = [(0,3),(2,3),(1,3),(2,4),(6,6),(3,3),(4,3)]

obs = [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
#waypoints =  RRT_hw.RRT([10,10],[0,0],[1,9],[[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [1,7], [2,7], [3,7]])
pathfinder.send_map_info_for_plot(10, obs, unknown)
waypoints =  pathfinder.findShortPath(10,[0,0],[1,9],obs)

for i in range(len(unknown)):
    obs.append(unknown[i])
    # unknown.remove(unknown[i])

    waypoints =  pathfinder.findShortPath(10,[0,i],[1,9],obs)
# print waypoints