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
C3 = 0.0
C4 = 0.0

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
	t_importance = nodes[:,4]
	rew_path = path[:]
	idle_path = idleness[list(set(rew_path))].sum()
	len_path = path_length(rew_path)
	shortest_path = shortest_path[path[0]]
	shortest_path = shortest_path[unvisited_nodes.astype('int')]
	idle_neighbor = -C2*(t_importance[list(set(rew_path))]).sum()-C3*len_path-C4*shortest_path.sum()
	return C1*idle_path + idle_neighbor


"""Time Period Based Patrolling"""
def tpbp(departed,vehicles,nodes,routes_list,path_list,shortest_path):
	#Populate the list of departed vehicles
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
		nodes.nodes[vehicles[int(veh_id)].dest_node,5] = 0	#Label destination nodes as visited
	#Begin algo
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]	
		#If the current vehicle is on its final edge
		if len(curr_vehicle.path)<2:
			nodes.nodes[curr_vehicle.path[0],1] = 0 #Reset its idleness = 0
			nodes.nodes[curr_vehicle.path[0],4] = nodes.nodes[curr_vehicle.path[0],3] #Reset the t_importance to time_period
			nodes.nodes[curr_vehicle.path[0],5] = 1 #Label the target node as unvisited to allow loop around the current node
			rew = None
			path_found = False
			#Find the unvisited nodes and sort them in order of the time importance (ascending)
			t_importance = nodes.nodes[np.where(nodes.nodes[:,5]==1)] #Unvisited
			t_importance = t_importance[t_importance[:,1].argsort()] #Sorted
			t_importance = t_importance[:,[0,4]] #Extracted columns containing node_id and time importance
			#Populating a list of node_ids with the values of time importance in the similar range as the least value of time importance
			indices = np.where(np.logical_and( t_importance[0][1]<=(t_importance[:,1]+1) , t_importance[0][1]>=(t_importance[:,1]-1))) 
			possible_targets = np.asarray(t_importance)[indices][:,0].astype('int')
			#Iterate till path is not found
			while path_found == False:
				rew = None
				for new_target in possible_targets:
					distance = nodes.nodes[:,4][new_target]
					for path in path_list :
						"""In the list of all possible paths, start_node should be current target, end node should be the new_target and 
						the path must contain at least 2 nodes and length of path must be less than the value of time_importance (distance)"""
						if (new_target == path[len(path)-1]) and len(path)>=2 and path[0]==curr_vehicle.path[0] and path_length(path)<distance:
							#Choose the one with the best reward
							if (rew == None or rew<reward(path,nodes.nodes,t_importance[:,0],routes_list,shortest_path)):
								path_found = True
								best_path = path[:]
								rew = reward(path,nodes.nodes,t_importance[:,0],routes_list,shortest_path)
				#If still path not found, choose the targets as the ones with higher values of time importance
				if path_found == False:
					t_importance = np.delete(t_importance,indices,axis=0)
					print("False",t_importance)
					indices = np.where( np.logical_and( t_importance[0][1]<=(t_importance[:,1]+1) , t_importance[0][1]>=(t_importance[:,1]-1) ) ) 
					possible_targets = np.asarray(t_importance)[indices][:, 0].astype('int')
				else:
					#Assign the best path and the destination node to the vehicle
					curr_vehicle.path = best_path
					curr_vehicle.dest_node = best_path[len(best_path)-1]
					#print("best_path",best_path)
					#Reset the idleness of the target to 0
					nodes.nodes[:,1][new_target] = 0
			nodes.nodes[curr_vehicle.dest_node,5] = 0 #Label destination as visited

		if curr_vehicle.curr_edge == curr_vehicle.dest_edge: #Keep changing each destination of the vehicle without assigning new target
			node_id = int(curr_vehicle.dest_edge[:curr_vehicle.dest_edge.find("to")]) #Node just visited
			curr_vehicle.changeTarget(str(curr_vehicle.path[0])+"to"+str(curr_vehicle.path[1])) #Set the new target (Neighbouring node)
			nodes.visited[node_id] = nodes.visited[node_id] + 1 #Increase the counter for the number of visits
			nodes.nodes[node_id,1] = 0
			curr_vehicle.path.pop(0) #Delete the already visited elements
	return nodes.nodes[:]

"""Highest Neighbourhood Idleness Patrolling"""
def hnip(departed, vehicles, nodes, routes_list, path_list, shortest_path):
	#Populate the list of departed vehicles
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
		#If current vehicle is on its final edge
		if curr_vehicle.curr_edge == curr_vehicle.dest_edge:
			node_id = int(curr_vehicle.dest_edge[:curr_vehicle.dest_edge.find("to")])
			curr_target = int(curr_vehicle.dest_edge[curr_vehicle.dest_edge.find("to")+2:])
			nodes.nodes[curr_target,1] = 0 #Set current target's idleness as 0 to prevent repeated visits
			target = None
			path = None
			for route in routes_list:
				#Searching in the neighbourhood
				if route.find("."+str(curr_target)+"to") != -1:
					#Choose the new target such that idleness of the node is max
					if target == None or (nodes.nodes[target, 1] < nodes.nodes[int(route[route.find("to")+2:-1]), 1]):
						path = route[1:len(route)-1]
						target = int(route[route.find("to")+2:-1])
			curr_vehicle.changeTarget(path) #Assign new path
			nodes.visited[node_id] = nodes.visited[node_id] + 1 #Increase the counter for the number of visits
			nodes.nodes[target,1] = 0 #Set the new target's idleness as 0 to prevent other robots from choosing the node
	return nodes.nodes[:]
