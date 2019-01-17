from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import numpy
import math

# we need to import python modules from the $SUMO_HOME/tools directory
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

routes_list = []
departed = []
num_nodes = 16
idle_weight = 0

class veh:
	def __init__(self):
		self.veh_id = 0
		self.idleness = [0]*num_nodes
		self.cost = [0]*num_nodes
		self.target_node = 1
		self.curr_node = 0
		self.destination = 1
		self.dest_edge = 1
		self.curr_edge = "0to1"

		#NEEDS TO BE CALLED AFTER EVERY STEP FOR EVERY VEHICLE
		def updateIdleness():
			for i in range(len(self.idleness)):
				self.idleness[i] = self.idleness[i] + 1000
		self.updateIdleness = updateIdleness

		#NEEDS TO BE CALLED IF A VEHICLE REACHES A NODE
		def setIdleness(node):
			self.idleness[node] = 0
		self.setIdleness = setIdleness

		#NEEDS TO BE CALLED WHEN ASSIGNING A NODE
		def updateCost():
			distance = [None]*num_nodes
			last_edges = traci.vehicle.getRoute(str(self.veh_id))
			last_edge = last_edges[len(last_edges)-1]
			for i in range(0,num_nodes):
				possible_edges = []
				for j in range(len(routes_list)):
					if routes_list[j].find("to"+str(i)+".") != -1:
						temp_edge = routes_list[j]
						possible_edges.append(temp_edge[1:len(temp_edge)-1])
				distance[i] = None
				for j in range(len(possible_edges)):
					traci.vehicle.changeTarget(str(self.veh_id),possible_edges[j])
					possible_edge_length = traci.lane.getLength(possible_edges[j]+"_0")
					if distance[i] == None or traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[j],possible_edge_length)<distance[i]:
						distance[i] = traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[j],possible_edge_length)
				traci.vehicle.changeTarget(str(self.veh_id),last_edge)
			for i in range(len(self.cost)):
				self.cost[i] = idle_weight*self.idleness[i] + (1-idle_weight)*distance[i]

		self.updateCost = updateCost

		#CHANGING TARGET
		def changeTarget():
			count = 0
			#UPDATE THE COST
			# self.updateCost()
			# for i in range(len(self.cost)):
			# 	print(i," ",self.cost[i])
			# #SET THE VALUES OF MINIMUM COST AND NODE TO REACH
			# max_cost = None
			# max_cost_node = None
			# for i in range(len(self.cost)):
			# 	if (max_cost == None or idleness[i]>max_cost) and i!=self.target_node:
			# 		max_cost = idleness[i]
			# 		max_cost_node = i
			# print("target_node",self.target_node)
			# print("max_cost_node ",max_cost_node)
			# #FIND THE ROUTES
			# possible_edges = []
			# for i in range(len(routes_list)):
			# 	if routes_list[i].find("to"+str(max_cost_node)+".") != -1:
			# 		temp_edge = routes_list[i]
			# 		possible_edges.append(temp_edge[1:len(temp_edge)-1])
			# print(possible_edges)
			# #COMPUTE THE MINIMUM
			# distance = None
			# route_to_take = None
			# last_edges = traci.vehicle.getRoute(str(self.veh_id))
			# last_edge = last_edges[len(last_edges)-1]
			# for i in range(len(possible_edges)):
			# 	traci.vehicle.changeTarget(str(self.veh_id),possible_edges[i])
			# 	possible_edge_length = traci.lane.getLength(possible_edges[i]+"_0")
			# 	if distance == None or traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[i],possible_edge_length)<distance:
			# 		distance = traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[i],possible_edge_length)
			# 		route_to_take = possible_edges[i]
			# traci.vehicle.changeTarget(str(self.veh_id),last_edge)
			# print("route_to_take ",route_to_take)
			# #SET THE TARGET
			# #print("last_edge ",last_edge)
			# if traci.vehicle.getRoadID(str(self.veh_id)) == last_edge and count == 0:
			# 	count = 1
			# 	print(traci.vehicle.getRoadID(str(self.veh_id)))
			# 	print("Changing destination")
			# 	self.destination = int(route_to_take[2+route_to_take.find("to"):])
			# 	traci.vehicle.changeTarget(str(self.veh_id),route_to_take)
			# 	print(self.destination)
			# 	self.target_node = int(traci.vehicle.getRoadID(str(self.veh_id))[traci.vehicle.getRoadID(str(self.veh_id)).find("to")+2:])
			# else:
			# 	count = 0
			if self.curr_edge != traci.vehicle.getRoadID(str(self.veh_id)) and traci.vehicle.getRoadID(str(self.veh_id)).find(":") == -1:
				node_visited = int(self.curr_edge[self.curr_edge.find("to")+2:])
				self.curr_edge = traci.vehicle.getRoadID(str(self.veh_id))
				self.setIdleness(node_visited)
				global idleness
				for j in range(num_nodes):
					if idleness[j][0] == node_visited:
						idleness[j][1] = 0
			#UPDATE THE COST
			self.updateCost()
		self.changeTarget = changeTarget

		def setTarget(node):
			possible_edges = []
			for i in range(len(routes_list)):
				if routes_list[i].find("to"+str(node)+".") != -1:
					temp_edge = routes_list[i]
					possible_edges.append(temp_edge[1:len(temp_edge)-1])
			#print(possible_edges)
			#COMPUTE THE MINIMUM
			distance = None
			route_to_take = None
			last_edges = traci.vehicle.getRoute(str(self.veh_id))
			last_edge = last_edges[len(last_edges)-1]
			for i in range(len(possible_edges)):
				traci.vehicle.changeTarget(str(self.veh_id),possible_edges[i])
				possible_edge_length = traci.lane.getLength(possible_edges[i]+"_0")
				if distance == None or traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[i],possible_edge_length)<distance:
					distance = traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[i],possible_edge_length)
					route_to_take = possible_edges[i]
			traci.vehicle.changeTarget(str(self.veh_id),route_to_take)
			self.destination = int(route_to_take[2+route_to_take.find("to"):])
			#print(self.destination)
			if traci.vehicle.getRoadID(str(self.veh_id)).find(":") == -1 and len(traci.vehicle.getRoadID(str(self.veh_id)))>1:
				print(traci.vehicle.getRoadID(str(self.veh_id)))
				self.target_node = int(traci.vehicle.getRoadID(str(self.veh_id))[traci.vehicle.getRoadID(str(self.veh_id)).find("to")+2:])
		self.setTarget = setTarget


def add_vehicle(id_veh,start_edge):
    traci.vehicle.add(id_veh,start_edge,speed=0)
    traci.vehicle.setMaxSpeed(id_veh,2.5)
    vehicle = veh()
    vehicle.veh_id = id_veh
    return vehicle

def run():
	#GENERATE ROUTE LIST
	######################################################
    lanes = traci.lane.getIDList()
    for i in range(0,len(lanes)):
        if lanes[i].find(":") == -1:
            routes_list.append("."+lanes[i][:-2]+".")
            start_node = lanes[i][:-2][:lanes[i].find("to")]
            end_node = lanes[i][:-2][lanes[i].find("to")+2:]
            traci.route.add(str(lanes[i][:-2]),[str(lanes[i][:-2])])

    ######################################################
    
    #INITIALIZE IDLENESS
    idleness = numpy.zeros(shape=(num_nodes,2))
    global idleness
    for i in range(0,num_nodes):
    	idleness[i][1] = 0
    	idleness[i][0] = i

    ######################################################
    
    #GENERATE VEHICLES
    vehicles = []
    vehicles.append(add_vehicle("0","0to1"))
    vehicles.append(add_vehicle("1","0to1"))
    ######################################################

    #PERFORM SIMULATION
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        curr_time = traci.simulation.getCurrentTime()
        traci.simulationStep()

        #ADD DEPARTED VEHICLES TO THE LIST
        for veh_id in traci.simulation.getDepartedIDList():
            departed.append(veh_id)

        #PERFORM COMPUTATION FOR EACH DEPARTED VEHICLE
        for veh_id in departed:
        	curr_vehicle = vehicles[int(veh_id)]
        	curr_vehicle.updateIdleness()
        	curr_vehicle.changeTarget()

        #UPDATE GLOBAL IDLENESS
        for i in range(len(idleness)):
        	idleness[i][1] = idleness[i][1] + 1000

        #SORT IDLENESS
        idleness = idleness[idleness[:,0].argsort()][::-1]

        #ASSIGN MAX_IDLENESS NODES
        max_idleness = idleness[0][1]
        curr_targets = [1]*len(vehicles)
        assigned = 0
        iterator = 0
        while iterator<num_nodes and (max_idleness == idleness[iterator][1] and assigned != len(vehicles)):
        	#print(assigned," ",max_idleness," ",idleness[iterator][1])
        	min_cost = None
        	node = int(idleness[iterator][0])
        	veh_to_assign = None
        	if node not in curr_targets:
        		# print(node," ",curr_targets)
	        	for i in range(len(vehicles)):
	        		if vehicles[i].cost[node]<min_cost or min_cost == None:
	        			min_cost = vehicles[i].cost[node]
	        			veh_to_assign = i 
	        	if veh_to_assign != None:
	        		curr_targets[veh_to_assign] = node
	        		assigned = assigned + 1
	        		vehicles[veh_to_assign].setTarget(node)
        		print(curr_targets)
        	iterator = iterator + 1

        step += 1

    ######################################################

    #CLOSE SIMULATION
    traci.close()
    sys.stdout.flush()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/hello.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
