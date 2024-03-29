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

global C1,C2,C3,C4,theta1,theta2,theta3
C1 = 1.0
C2 = 0.5
C3 = 2.0
C4 = 10.0
theta1 = 1.0
theta2 = -0.75
theta3 = -5.0

def path_length(path):
	path_length = 0
	for i in range(0,len(path)-1):
		start_node = path[i]
		end_node = path[i+1]
		path_length = path_length + traci.lane.getLength(str(start_node)+"to"+str(end_node)+"_0")/100
	return path_length


def TPP_Reward(path,nodes,unvisited_nodes,routes_list,shortest_path):
	idleness = nodes[:,1]
	time_period = nodes[:,3]
	rew_path = path[:]
	idle_path = idleness[list(set(rew_path))].sum()
	len_path = path_length(rew_path)
	shortest_path = shortest_path[path[0]]
	shortest_path = shortest_path[unvisited_nodes.astype('int')]
	idle_neighbor = C2*(time_period[list(set(rew_path))]).sum()-C3*len_path-C4*shortest_path.sum()
	return C1*idle_path + idle_neighbor

def DTAG_Reward(current,target,nodes,vehicle):
	target_x = traci.junction.getPosition(str(target))[0]
	target_y = traci.junction.getPosition(str(target))[1]
	curr_x = traci.junction.getPosition(str(current))[0]
	curr_y = traci.junction.getPosition(str(current))[1]
	origin_x = traci.junction.getPosition(str(vehicle.origin))[0]
	origin_y = traci.junction.getPosition(str(vehicle.origin))[1]
	origin_dist = ((origin_x-target_x)**2+(origin_y-target_y)**2)**0.5
	path_dist = ((curr_x-target_x)**2+(curr_y-target_y)**2)**0.5
	return theta1*nodes.nodes[target,1]*5.652 + theta2*path_dist*0.065 + theta3*origin_dist*0.065


"""Time-Period Patrolling"""
def TPP(departed,vehicles,nodes,routes_list,path_list,shortest_path):
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
			indices = np.where(np.logical_and( t_importance[0][1]<=(t_importance[:,1]+0.1) , t_importance[0][1]>=(t_importance[:,1]-0.1))) 
			possible_targets = np.asarray(t_importance)[indices][:,0].astype('int')
			#Iterate till path is not found
			while path_found == False:
				rew = None
				for new_target in possible_targets:
					distance = nodes.nodes[new_target,4]
					for path in path_list :
						"""In the list of all possible paths, start_node should be current target, end node should be the new_target and 
						the path must contain at least 2 nodes and length of path must be less than the value of time_importance (distance)"""
						if (new_target == path[len(path)-1]) and len(path)>=2 and path[0]==curr_vehicle.path[0] and path_length(path)<distance:
							#Choose the one with the best reward
							if (rew == None or rew<TPP_Reward(path,nodes.nodes,t_importance[:,0],routes_list,shortest_path)):
								path_found = True
								best_path = path[:]
								rew = TPP_Reward(path,nodes.nodes,t_importance[:,0],routes_list,shortest_path)
				#If still path not found, choose the targets as the ones with higher values of time importance
				if path_found == False:
					t_importance = np.delete(t_importance,indices,axis=0)
					indices = np.where(np.logical_and(t_importance[0][1]<=(t_importance[:,1]+0.2),t_importance[0][1]>=(t_importance[:,1]-0.2))) 
					possible_targets = np.asarray(t_importance)[indices][:,0].astype('int')
				else:
					#Assign the best path and the destination node to the vehicle
					curr_vehicle.path = best_path
					curr_vehicle.dest_node = best_path[len(best_path)-1]
					#Reset the idleness of the target to 0
					nodes.nodes[new_target,1] = 0
			nodes.nodes[curr_vehicle.dest_node,5] = 0 #Label destination as visited

		if curr_vehicle.curr_edge == curr_vehicle.dest_edge: #Keep changing each destination of the vehicle without assigning new target
			curr_vehicle.changeTarget(str(curr_vehicle.path[0])+"to"+str(curr_vehicle.path[1])) #Set the new target (Neighbouring node)
			curr_vehicle.path.pop(0) #Delete the already visited elements
	return nodes.nodes[:]


"""Conscientious Reactive Patrolling"""
def CRP(departed, vehicles, nodes, routes_list, path_list, shortest_path):
	#Populate the list of departed vehicles
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
	#Begin algo
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
		#If current vehicle is on its final edge
		if curr_vehicle.curr_edge == curr_vehicle.dest_edge:
			curr_target = int(curr_vehicle.dest_edge[curr_vehicle.dest_edge.find("to")+2:]) #Node ID with the current target
			nodes.nodes[curr_target,1] = 0 #Set current target's idleness as 0 to prevent repeated visits
			target = None
			end_edge = None
			for route in routes_list:
				#Searching in the neighbourhood
				if route.find("."+str(curr_target)+"to") != -1:
					#Choose the new target such that idleness of the node is max
					if target == None or (nodes.nodes[target,1] < nodes.nodes[int(route[route.find("to")+2:-1]),1]):
						end_edge = route[1:len(route)-1]
						target = int(route[route.find("to")+2:-1])
			curr_vehicle.changeTarget(end_edge) #Assign new path
			nodes.nodes[target,1] = 0 #Set the new target's idleness as 0 to prevent other robots from choosing the node
	return nodes.nodes[:]


"""DTAG Patrolling"""
def DTAG(departed, vehicles, nodes, routes_list, path_list, shortest_path):
    #Populate the list of departed vehicles
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
	#Begin algo
	for veh_id in departed:
		curr_vehicle = vehicles[int(veh_id)]
		#If current vehicle is on its final edge
		if curr_vehicle.curr_edge == curr_vehicle.dest_edge:
			curr_target = int(curr_vehicle.dest_edge[curr_vehicle.dest_edge.find("to")+2:]) #Node ID with the current target
			nodes.nodes[curr_target,1] = 0 #Set current target's idleness as 0 to prevent repeated visits
			target = None
			reward = None
			#Compute Arg Max of Utility Function
			for route in routes_list:
				if (target == None or (reward < DTAG_Reward(curr_target,int(route[route.find("to")+2:-1]),nodes,curr_vehicle))) and route[1:-1]!=curr_vehicle.curr_edge:
					end_edge = route[1:len(route)-1]
					target = int(route[route.find("to")+2:-1])
					reward = DTAG_Reward(curr_target,int(route[route.find("to")+2:-1]),nodes,curr_vehicle)
			curr_vehicle.changeTarget(end_edge) #Assign new path
			#Check for clashes with other vehicles
			##############################################################
			for clash_veh_id in departed:
				clash_vehicle = vehicles[int(clash_veh_id)]
				clash_target = clash_vehicle.dest_edge[clash_vehicle.dest_edge.find("to")+2:]
				clash_reward = None
				if target == clash_target and clash_veh_id != veh_id:
					#Compute Arg Max of Utility Function
					clash_curr_location = clash_vehicle.curr_edge[clash_vehicle.curr_edge.find("to")+2:]
					for route in routes_list:
						if clash_target == None or clash_reward<DTAG_Reward(clash_curr_location,int(route[route.find("to")+2:-1]),nodes,clash_vehicle):
							clash_dest_edge = route[1:len(route)-1]
							clash_target = int(route[route.find("to")+2:-1])
							clash_reward = DTAG_Reward(clash_curr_location,int(route[route.find("to")+2:-1]),nodes,clash_vehicle)
					clash_vehicle.changeTarget(clash_dest_edge) #Assign new path
			###############################################################
			nodes.nodes[target,1] = 0 #Set the new target's idleness as 0 to prevent other robots from choosing the node
	return nodes.nodes[:]

""""Cooperative Patrolling Grid"""

def CPG(departed, vehicles, nodes, routes_list, path_list, shortest_path):
	veh0 = vehicles[0]
	veh1 = vehicles[1]
	veh2 = vehicles[2]

	if veh0.curr_edge == "1to0":
		veh0.changeTarget("5to6")
	elif veh0.curr_edge == "5to6":
		veh0.changeTarget("1to0")
	
	if veh1.curr_edge == "8to7" and veh1.origin > 0:
		veh1.changeTarget("11to12")
	elif veh1.curr_edge == "6to5":
		veh1.changeTarget("11to12")
	elif veh1.curr_edge == "11to12":
		if veh1.origin > 0:
			veh1.changeTarget("6to5")
		elif veh1.origin < 0:
			veh1.changeTarget("8to7")
		veh1.origin = -veh1.origin

	if veh2.curr_edge == "15to10":
		veh2.changeTarget("1to0")
	elif veh2.curr_edge == "1to0":
		veh2.changeTarget("15to10")

	return nodes.nodes[:]


""""Cooperative Patrolling Random"""


def CPR(departed, vehicles, nodes, routes_list, path_list, shortest_path):
	veh0 = vehicles[0]
	veh1 = vehicles[1]
	if veh0.curr_edge == veh0.dest_edge:
		if veh0.curr_edge == "4to1":
			veh0.changeTarget("4to3")
		elif veh0.curr_edge == "4to3":
			if veh0.origin > 0:
				veh0.changeTarget("9to8")
			else:
				veh0.changeTarget("4to1")
			veh0.origin = -veh0.origin
		elif veh0.curr_edge == "9to8":
			if veh0.origin < 0:
				veh0.changeTarget("5to2")
			else:
				veh0.changeTarget("4to3")
			veh0.origin = -veh0.origin
		elif veh0.curr_edge == "5to2":
			veh0.changeTarget("9to8")

	if veh1.curr_edge == veh1.dest_edge:
		if veh1.curr_edge == "2to7":
			veh1.changeTarget("2to0")
		elif veh1.curr_edge == "2to0":
			veh1.changeTarget("4to1")
		elif veh1.curr_edge == "4to1":
			veh1.changeTarget("2to7")

	return nodes.nodes[:]
