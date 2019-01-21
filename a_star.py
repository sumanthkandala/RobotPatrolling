############################################
#A* ALGORITHM

# def get_path(queue,target,global_nodes,unvisited_nodes,routes_list):
# 	path = []
# 	nodes = queue.nodes
# 	distance = queue.distance
# 	curr_reward = queue.reward
# 	start_node = nodes[0]
# 	if distance[0] > 0:
# 		for edge in routes_list:
# 			if edge.find(":") == -1 and edge.find("."+str(start_node)+"to") != -1:
# 				queue.path.append(int(edge[len("."+str(start_node)+"to"):-1]))
# 				queue.distance.append(distance[0] - traci.lane.getLength(edge[1:-1]+"_0"))
# 				start_edges = []
# 				end_edges = []
# 				for lane in routes_list:
# 					if lane.find(":") == -1 and lane.find("."+str(start_node)+"to") != -1:
# 						start_edges.append(lane)
# 					if lane.find(":") == -1 and lane.find("to"+str(target)+".") != -1:
# 						end_edges.append(lane)
# 				route = None
# 				dist = None
# 				for start in start_edges:
# 					for end in end_edges:
# 						traci.vehicle.add("cost_computer",start,speed=0)
# 						traci.vehicle.changeTarget("cost_computer",end)
# 						if dist==None or traci.vehicle.getDrivingDistance("cost_computer",end,traci.lane.getLength(end+"_0"))<dist:
# 							start_edge = start
# 							end_edge = end
# 						traci.vehicle.remove("cost_computer")
# 				traci.vehicle.add("cost_computer",start_edge,speed=0)
# 				traci.vehicle.changeTarget("cost_computer",end_edge)			
# 				route = traci.vehicle.getRoute("cost_computer")
# 				path = []
# 				for lane in route:
# 					path.append(int(lane[:lane.find("to")]))
# 					print(int(lane[:lane.find("to")]))
# 				reward_to_go = reward(path,global_nodes,unvisited_nodes,routes_list)
# 				queue.reward.append(curr_reward + reward)
# 		#DISTANCE SORT
# 		nodes = nodes[np.where(distance>0)]
# 	return path_tree

#def dist(start_node,end_node):