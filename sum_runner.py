from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import os
import sys
import optparse
import subprocess
import csv
import random
import numpy
import math
import statistics
from scipy.fftpack import fft, ifft
from matplotlib import pyplot

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
import pandas as pd
from pandas.tools.plotting import autocorrelation_plot

simulation_time = 3000
routes_list = []
departed = []
max_speed = 9
num_vehicles = 0
num_nodes = 9
visited_nodes = []
counter = []
idleness = numpy.zeros(shape=(num_nodes,2))
avg_idleness = [0]*num_nodes
global avg_idleness,visited_nodes
global idleness

################################################################

#CLASS FOR EACH VEHICLE
def estimated_autocorrelation(x):
    n = len(x)
    variance = x.var()
    x = x-x.mean()
    r = numpy.correlate(x, x, mode = 'full')[-n:]
    #assert N.allclose(r, N.array([(x[:n-k]*x[-(n-k):]).sum() for k in range(n)]))
    result = r/(variance*(numpy.arange(n, 0, -1)))

    return result

class veh:
	def __init__(self,curr_edge):
		self.veh_id = 0
		self.idleness = [0]*num_nodes
		self.idleness[0] = 0
		self.dist = [0]*num_nodes
		self.cost = [0]*num_nodes
		self.target_node = 1
		self.curr_node = 0
		self.destination = 1
		self.dest_edge = curr_edge
		self.curr_edge = curr_edge
		self.count = 0

		#NEEDS TO BE CALLED AFTER EVERY STEP FOR EVERY VEHICLE
		def updateIdleness():
			for i in range(len(self.idleness)):
				self.idleness[i] = self.idleness[i] + 15
		
		self.updateIdleness = updateIdleness

		#NEEDS TO BE CALLED IF A VEHICLE REACHES A NODE
		def setIdleness(node):
			self.idleness[node] = 0
		
		self.setIdleness = setIdleness

		#NEEDS TO BE CALLED WHEN ASSIGNING A NODE
		def updateCost():
			distance = [None]*num_nodes
			#last_edges = traci.vehicle.getRoute(str(self.veh_id))
			#slast_edge = last_edges[len(last_edges)-1]
			#print("Vehicle ID ",self.veh_id)
			#OBTAINING MINIMUM DISTANCE TO EACH NODE
			preffered_edge = ["0to1"]*num_nodes
			for i in range(0,num_nodes):
				
				#OBTAINING POSSIBLE ROUTES TO THE NODE
				possible_edges = []
				for j in range(len(routes_list)):
					if routes_list[j].find("to"+str(i)+".") != -1:
						temp_edge = routes_list[j]
						possible_edges.append(temp_edge[1:len(temp_edge)-1])
				
				#OBTAINS THE DISTANCE TO EACH OF THE POSSIBLE ROUTE FOR THAT NODE AND SETS THE DISTANCE AS THE MINIMUM
				#past_route = traci.vehicle.getRoute(str(self.veh_id))
				distance[i] = None
				for j in range(len(possible_edges)):
					traci.vehicle.changeTarget(str(self.veh_id),possible_edges[j])
					possible_edge_length = traci.lane.getLength(possible_edges[j]+"_0")
					if distance[i] == None or traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[j],possible_edge_length)<distance[i]:
						distance[i] = traci.vehicle.getDrivingDistance(str(self.veh_id),possible_edges[j],possible_edge_length)
						preffered_edge[i] = possible_edges[j]
					#future_route = traci.vehicle.getRoute(str(self.veh_id))
					#print("asodshoasdhsoads ",future_route," sds ",past_route) 
					traci.vehicle.changeTarget(str(self.veh_id),self.dest_edge)
				#print("Distance ",distance)

			#NORMALIZE DISTANCES
			max_dist = None
			for i in range(0,num_nodes):
				if max_dist == None or max_dist<distance[i]:
					max_dist = distance[i]
				
			#NORMALIZE IDLENESS
			max_idleness = None	
			for i in range(0,num_nodes):
				#print(i , self.idleness[i])
				if max_idleness == None or max_idleness<self.idleness[i]:
					max_idleness = self.idleness[i]

			#GET DISTANCE FROM ORIGIN AND NORMALIZE IT
			dist_origin = [0]*num_nodes
			for i in range(0,num_nodes):
				dist_origin[i] = traci.simulation.getDistanceRoad("0to1",0,preffered_edge[i],traci.lane.getLength(preffered_edge[i]+"_0"),False)
				#dist_origin[i] = traci.vehicle.getDrivingDistance(str(self.veh_id),preffered_edge[i],traci.lane.getLength(preffered_edge[i]+"_0"))
			#NORMALIZE IT
			max_dist_org = None
			for i in range(0,num_nodes):
				if max_dist_org == None or max_dist_org<dist_origin[i]:
					max_dist_org = dist_origin[i]

			#UPDATING COST OF EACH NODE
			for i in range(len(self.cost)):
				self.dist[i] = distance[i]
				#print("idleness of ",self.idleness[i]/100,"Distance to ",i," is ",self.dist[i]/10)
				self.cost[i] = self.idleness[i]# - 0.75*distance[i]/10.0 #- 5.0*dist_origin[i]/10.0
				#print(i," ",self.cost[i])
				#self.cost[i] = idle_weight/(1000+int(idleness[j][1])) + (1-idle_weight)*distance[i]
		
		self.updateCost = updateCost

		#FUNCTION TO CHANGE THE TARGET OF THE VEHICLE
		def changeTarget(min_dist_node):
			#FIND THE ROUTES TO THE TARGET
			possible_edges = []
			#print(traci.vehicle.getRoadID(str(self.veh_id )))
			if traci.vehicle.getRoadID(str(self.veh_id)).find(":") == -1:
				for i in range(len(routes_list)):
					if routes_list[i].find("to"+str(min_dist_node)+".") != -1:
						temp_edge = routes_list[i]
						possible_edges.append(temp_edge[1:len(temp_edge)-1])
				#print(possible_edges)
				
				#COMPUTE THE MINIMUM DISTANCE ROUTE
				distance = 1000000000
				route_to_take = None
				#last_edges = traci.vehicle.getRoute(str(self.veh_id))
				#last_edge = last_edges[len(last_edges)-1]
				temp = traci.vehicle.getRoadID(str(self.veh_id))
				if temp.find(":") == -1:
					for i in range(len(possible_edges)):
						position = traci.vehicle.getPosition(str(self.veh_id))
						#print("position ",position)
						traci.vehicle.changeTarget(str(self.veh_id),possible_edges[i])
						possible_edge_length = traci.lane.getLength(possible_edges[i]+"_0")
						#print(traci.vehicle.getRoute(str(self.veh_id)))
						#print(possible_edges[i]," distance ",traci.simulation.getDistanceRoad(self.curr_edge,traci.lane.getLength(self.curr_edge+"_0"),possible_edges[i],possible_edge_length))
						val = traci.simulation.getDistanceRoad(self.curr_edge,traci.lane.getLength(self.curr_edge+"_0"),possible_edges[i],possible_edge_length)
						if distance == None or val<distance:
							distance = val
							route_to_take = possible_edges[i]
							#print("Route to take",route_to_take," distance ",distance,traci.vehicle.getDrivingDistance(str(self.veh_id),"7to8",traci.lane.getLength("7to8_0")-10))
					traci.vehicle.changeTarget(str(self.veh_id),self.curr_edge)
					#print("route_to_take ",traci.vehicle.getRoute(str(self.veh_id)))
					
					#SET THE TARGET
					self.count = 1
					#print(traci.vehicle.getRoadID(str(self.veh_id)))
					#print("Changing destination")
					self.destination = int(route_to_take[2+route_to_take.find("to"):])
					#print("Changing target for ",self.veh_id,self.destination,route_to_take)
					traci.vehicle.changeTarget(self.veh_id,route_to_take)
					#print(traci.vehicle.getRoute(str(self.veh_id)))
					self.dest_edge = route_to_take
					if len(traci.vehicle.getRoadID(str(self.veh_id))) >1:
						self.target_node = int(traci.vehicle.getRoadID(str(self.veh_id))[traci.vehicle.getRoadID(str(self.veh_id)).find("to")+2:])
					
					#UPDATE THE COST
					self.updateCost()
		
		self.changeTarget = changeTarget

		####################################################

		#UPDATE CURR_EDGE AND CURRENT DESTINATION AND SET IDLENESS OF VISITED NODE
		def updateParams():
			global idleness,counter
			
			#UPDATE THE LOCAL IDLENESS
			for i in range(0,num_nodes):
				self.idleness[i] = self.idleness[i] + 15
			#self.updateCost()
			temp = traci.vehicle.getRoadID(str(self.veh_id))
			#CHECK IF CURRENT EDGE HAS CHANGED AND IF CURRENT EDGE IS NOT A JUNCTION (AUTOMATIC EDGES GENERATED BY TRACI JOINING TWO OR MORE USER DEFINED EDGES)
			if self.curr_edge != temp and temp.find(":") == -1 and len(temp)>1:
				
				#OBTAIN THE VALUE OF VISITED NODE
				node_visited = int(self.curr_edge[self.curr_edge.find("to")+2:])
				self.curr_edge = temp

				#SET THE LOCAL IDLENESS OF THE VISITED NODE AS 0
				self.setIdleness(node_visited)
				global visited_nodes
				visited_nodes.append(node_visited)
				counter[node_visited] = counter[node_visited] + 1 
				#SET THE GLOBAL IDLENESS OF THE VISITED NODE AS 0
				for i in range(len(idleness)):
					if idleness[i][0] == node_visited:
						idleness[i][1] = 0
				self.count = 0
				# dtag_assign(self.veh_id,node_visited)

		self.updateParams = updateParams

###########################################################

#FUNCTION TO ADD VEHICLE STARTING EDGE
def add_vehicle(start_edge):
	global num_vehicles
	#ASSIGNS THE VEHICLE ID AND INITIAL SPEED AS 0
	traci.vehicle.add(str(num_vehicles),start_edge,speed=0)
	#SETS THE MAXIMUM SPEED AS 2.5
	traci.vehicle.setMaxSpeed(str(num_vehicles),max_speed)
	#traci.vehicle.setActionStepLength(str(num_vehicles),1,True)
	vehicle = veh(start_edge)
	vehicle.veh_id = str(num_vehicles)
	#INCREMENT THE GLOBAL VALUE OF TOTAL NUMBER OF VEHICLES
	num_vehicles = num_vehicles + 1
	return vehicle

#DTAG ASSIGN

def dtag_assign(curr_vehicle,visited_node):
	#print("#############################################")
	print("Using DTAG Algorithm for ",curr_vehicle)
	global vehicles
	#UPDATE THE VISITED NODE IDLENESS
	vehicles[curr_vehicle].idleness[visited_node] = 0
	
	#FIND THE MAXIMUM COST(UTILITY) NODE
	max_cost = None
	target_node = None
	for i in range(0,num_nodes):
		if (max_cost == None or max_cost<vehicles[curr_vehicle].cost[i]) and i!=vehicles[curr_vehicle].destination:
			max_cost = vehicles[curr_vehicle].cost[i]
			target_node = i

	#CHANGE THE TARGET IDLENESS
	vehicles[curr_vehicle].idleness[target_node] = 0

	#COMMUNICATE TO OTHER NODES
	for veh_id in departed:
		if int(veh_id) != curr_vehicle:
			
			#UPDATE THE IDLENESS
			for i in range(0,num_nodes):
				vehicles[int(veh_id)].idleness[i] = min(vehicles[curr_vehicle].idleness[i],vehicles[int(veh_id)].idleness[i])
			
			#CONFLICT CASE:
			if curr_dest[int(veh_id)] == target_node:
				print("happening")
				maximum = None
				target = None
				curr_node = []
				for i in range(0,num_vehicles):
					curr_node.append(int(vehicles[i].curr_edge[vehicles[i].curr_edge.find("to")+2:]))
				print(curr_node)
				for i in range(0,num_nodes):
					if (maximum == None or maximum<vehicles[curr_vehicle].cost[i]) and i != visited_node and i not in curr_node:
						maximum = vehicles[curr_vehicle].cost[i]
						target = i
				curr_dest[int(veh_id)] = target
				vehicles[int(veh_id)].changeTarget(target)
				print(target)

	#print("New Target ",target_node)
	curr_dest[curr_vehicle] = target_node
	vehicles[curr_vehicle].changeTarget(target_node)
	#print(target_node)

##########################################################

#ACTUAL SIMULATION FUNCTION
def run():
	with open('idleness_long.csv','w') as file:
		writer = csv.writer(file)
		writer.writerow(['node','step','idleness','limit'])
		#GENERATE ROUTE LIST
		######################################################
		print(traci.junction.getIDCount())
		lanes = traci.lane.getIDList()
		for i in range(0,len(lanes)):
		    if lanes[i].find(":") == -1:
		        routes_list.append("."+lanes[i][:-2]+".")
		        start_node = lanes[i][:-2][:lanes[i].find("to")]
		        end_node = lanes[i][:-2][lanes[i].find("to")+2:]
		        traci.route.add(str(lanes[i][:-2]),[str(lanes[i][:-2])])
		######################################################

		#INITIALIZE IDLENESS
		for i in range(0,num_nodes):
			global idleness
			idleness[i][1] = 0
			idleness[i][0] = i

		######################################################

		#GENERATE VEHICLES
		vehicles = []
		global vehicles,counter
		vehicles.append(add_vehicle("1to0"))
		vehicles.append(add_vehicle("5to2"))
		vehicles.append(add_vehicle("3to6"))
		#vehicles.append(add_vehicle("0to1"))
		vehicles.append(add_vehicle("7to8"))
		curr_dest = [1]*num_vehicles
		curr_edge = [1]*num_vehicles
		idle_plot = numpy.zeros(shape=(num_nodes,simulation_time))
		counter = [0]*num_nodes
		global curr_edge,curr_dest
		######################################################
		end = False
		#PERFORM SIMULATION
		step = 0
		curr_time = traci.simulation.getCurrentTime()
		while traci.simulation.getMinExpectedNumber() > 0 and step<simulation_time:
		    prev_time = curr_time
		    curr_time = traci.simulation.getCurrentTime()
		    traci.simulationStep()

		    #ADD DEPARTED VEHICLES TO THE LIST
		    for veh_id in traci.simulation.getDepartedIDList():
		    	print(num_vehicles, " ",veh_id)
		    	departed.append(veh_id)

		    #UPDATE GLOBAL IDLENESS AND PRINTS TARGET AND IDLENESS
		    global idleness
		    #print(curr_dest)
		    #print(".......................................................")
		    global_avg_idleness = 0
		    global_idleness_max = 0
		    idleness_array = [0]*num_nodes
		    for i in range(len(idleness)):
		    	d = idleness[i][0],step,idleness[i][1]
		    	writer.writerow(d)
		    	idleness[i][1] = idleness[i][1] + 15
		    	avg_idleness[int(idleness[i][0])] = (avg_idleness[int(idleness[i][0])]*step + idleness[i][1])/(step+1)
		    	idleness_array[i] = avg_idleness[int(idleness[i][0])]
		    	idle_plot[i][int(step/10)] = idleness[i][1]
		    	#print(idleness[i][0]," ",idleness[i][1]/1000)
		    #print("step ",step," average_idleness ",numpy.mean(idleness_array)," max_idleness ",max(idleness_array)," std_dev ",numpy.std(idleness_array))
		    #print("#######################################################")

		    #SORT IDLENESS
		    idleness = idleness[numpy.lexsort((idleness[:,0],idleness[:,1]))][::-1]


		    #PERFORM COMPUTATION FOR EACH DEPARTED VEHICLE
		    count = 0
		    for veh_id in departed:
		    	curr_vehicle = vehicles[int(veh_id)]
		    	#curr_vehicle.updateIdleness()
		    	curr_vehicle.updateParams()
		    	curr_vehicle.updateCost()

		    for veh_id in departed:
		    	curr_vehicle = vehicles[int(veh_id)]
		    	#print(traci.vehicle.getRoute(veh_id))
		    	#print("Lane position", traci.vehicle.getLanePosition(veh_id)," Lane length ")
		    	#time_remaining()
		    	if curr_vehicle.curr_edge == curr_vehicle.dest_edge: #and traci.vehicle.getLanePosition(veh_id)>traci.lane.getLength(curr_vehicle.curr_edge+"_0")-2*max_speed:
		    		count = 1
		    		print(numpy.mean(idleness,axis=0)[1])
		    		dtag_assign(int(veh_id),int(curr_vehicle.curr_edge[curr_vehicle.curr_edge.find("to")+2:]))
		    		#print("veh_id",veh_id,vehicles[int(veh_id)].curr_edge,vehicles[int(veh_id)].dest_edge)

		    step += 1
		    end = True
		    for count in counter:
		    	if count>50:
		    		end = end&True
		    	else:
		    		end = end&False
		pyplot.show()
		######################################################
		route = []
		for i in range(0,num_vehicles):
			route.append(traci.vehicle.getRoute(str(i)))
		traci.close()
		with open('output.csv', 'w') as f:
			thewriter = csv.writer(f)
			thewriter.writerow(['vehicle','route'])
			for i in range(0,num_vehicles):
				for j in range(len(route[i])):
					thewriter.writerow([i+1,route[i][j]])
		global visited_nodes
		sys.stdout.flush()

##########################################################

#SIMULATION OPTIONS
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
    traci.start([sumoBinary, "-c", "data/hello.sumocfg"])
    #call the simulation function
    run()
