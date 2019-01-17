import vehicle
import nodes
import numpy as np
import os
import sys
import optparse
import subprocess
import numpy
import math
import csv
import random

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
import traci.constants as tc
import vehicleControl

global C1,C2,C3,C4
C1 = 1.0
C2 = 0.0
C3 = 10.0
C4 = 0.0

# class Queue:
# 	def __init__(self):
# 		self.nodes = []
# 		self.distance = []
# 		self.reward = []
# class tree:
# 	def __init__(self):
# 		self.path = []
# 		self.distance = None

def sort_time(nodes,curr_node):
	nodes[int(curr_node)][4] = 1
	unvisited_nodes = nodes[np.where(nodes[:,4] == 0)]
	time_period = unvisited_nodes[:,3]
	idleness = unvisited_nodes[:,1]
	unvisited_nodes[:,1] = time_period - idleness
	unvisited_nodes = unvisited_nodes[unvisited_nodes[:,1].argsort()]
	nodes[int(curr_node)][4] = 0
	new_targets = list(unvisited_nodes[:,0].astype(int))
	new_targets.append(int(curr_node))
	return np.asarray(new_targets)

# def get_path_list(Tree,routes_list):
# 	path_tree = []
# 	start_node = Tree.path
# 	distance = Tree.distance
# 	if distance > 0:
# 		for edge in routes_list:
# 			if edge.find(":") == -1 and edge.find("."+str(start_node[len(start_node)-1])+"to") != -1:
# 				temp = tree()
# 				temp.path.extend(start_node)
# 				temp.path.append(int(edge[len("."+str(start_node[len(start_node)-1])+"to"):-1]))
# 				temp.distance = distance - traci.lane.getLength(edge[1:-1]+"_0")
# 				#print("temp.path ",temp.path,"temp.distance",temp.distance)
# 				temp_tree = get_path_list(temp,routes_list)
# 				for added_path in temp_tree:
# 					path_tree.append(added_path)
# 				path_tree.append(temp.path)
# 	return path_tree

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


def path_length(path):
	path_length = 0
	for i in range(0,len(path)-1):
		start_node = path[i]
		end_node = path[i+1]
		path_length = path_length + traci.lane.getLength(str(start_node)+"to"+str(end_node)+"_0")/10
	return path_length


def reward(path,nodes,unvisited_nodes,routes_list,shortest_path):
	idle_path = 0
	idle_neighbor = 0
	idleness = nodes[:,1]
	time_period = nodes[:,3]
	rew_path = path[:]
	idle_path = idleness[list(set(rew_path))].sum()
	len_path = path_length(rew_path)
	idle_neighbor = (-C2*(time_period[list(set(rew_path))]-idleness[list(set(rew_path))])-C3*len_path).sum()-C4*shortest_path[np.where(shortest_path[0] == path[len(path)-1]),:].sum()
	return C1*idle_path + idle_neighbor

def algo(departed,vehicles,nodes,routes_list,path_list,shortest_path):
	targets = []
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
		nodes.nodes[vehicles[int(veh_id)].dest_node,4] = 1	
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]	
		if len(curr_vehicle.path)<2:
			nodes.nodes[curr_vehicle.path[len(curr_vehicle.path)-1],4] = 0
			unvisited_nodes = sort_time(nodes.nodes,nodes.nodes[:,0][curr_vehicle.path[len(curr_vehicle.path)-1]])
			rew = None
			path_found = False
			first_nodes = [] 
			t_importance = nodes.nodes[unvisited_nodes,:]
			t_importance[:,1] = t_importance[:,3] - t_importance[:,1]
			t_importance = t_importance[:,[0,1]]
			print(veh_id,unvisited_nodes)
			indices = np.where(np.logical_and( t_importance[0][1]<=(t_importance[:,1]+1) , t_importance[0][1]>=(t_importance[:,1]-1))) 
			possible_targets = np.asarray(unvisited_nodes)[indices]
			while path_found == False:
				rew = None
				for new_target in possible_targets:
					distance = (nodes.nodes[:,3][new_target]-nodes.nodes[:,1][new_target])
					for path in path_list :
						if (new_target == path[len(path)-1]) and len(path)>=2 and path[0]==curr_vehicle.path[0] and path_length(path)<distance:
							#print(path,reward(path,nodes.nodes,unvisited_nodes,routes_list,shortest_path))
							if (rew == None or rew<reward(path,nodes.nodes,unvisited_nodes,routes_list,shortest_path)):
								path_found = True
								best_path = path[:]
								rew = reward(path,nodes.nodes,unvisited_nodes,routes_list,shortest_path)
				if path_found == False:
					unvisited_nodes = np.delete(unvisited_nodes,indices)
					t_importance = nodes.nodes[unvisited_nodes,:]
					t_importance[:,1] = t_importance[:,3] - t_importance[:,1]
					t_importance = t_importance[:,[0,1]]
					print(t_importance)
					indices = np.where( np.logical_and( t_importance[0][1]<=(t_importance[:,1]+1) , t_importance[0][1]>=(t_importance[:,1]-1) ) ) 
					possible_targets = unvisited_nodes[indices] 
				else:
					curr_vehicle.path = best_path
					curr_vehicle.dest_node = best_path[len(best_path)-1]
					print("veh_id",veh_id,"path",best_path,curr_vehicle.dest_node)	
					nodes.nodes[:,1][new_target] = 0
			nodes.nodes[curr_vehicle.dest_node,4] = 1

		if curr_vehicle.curr_edge == curr_vehicle.dest_edge: #Keep changing each destination of the vehicle without assigning new target
			node_id = int(curr_vehicle.dest_edge[:curr_vehicle.dest_edge.find("to")])
			curr_vehicle.changeTarget(str(curr_vehicle.path[0])+"to"+str(curr_vehicle.path[1]))
			nodes.visited[node_id] = nodes.visited[node_id] + 1
			nodes.nodes[node_id,1] = 0
			curr_vehicle.path.pop(0)
	return nodes.nodes[:]