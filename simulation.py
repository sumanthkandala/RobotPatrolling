import vehicle
import patrolling
import numpy as np
import optparse
import sys
import os
import json
import matplotlib.pyplot as plt
from patrolling import path_length
import seaborn as sns
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
import nodes
import vehicle
import csv
from tqdm import tqdm
import time

class tree:
	def __init__(self):
		self.path = []
		self.distance = None

class simulation:
	def __init__(self):

		#Simulation params
		self.departed = []
		self.routes = []
		self.vehicles = []
		self.routes = []
		self.paths = []
		self.speed = 10
		self.curr_time = 0
		self.total_time = 500
		self.node = nodes.node()
		self.max_x = 0
		self.max_y = 0
		self.min_x = 0
		self.min_y = 0
		self.heatmap = []
		self.size = 100
		self.shortest_path = self.size*((len(self.node.nodes))**2)*np.ones((len(self.node.nodes),len(self.node.nodes)))
		#Generate routes
		def gen_route():
			self.lanes = traci.lane.getIDList()
			for lane in self.lanes:
			    if lane.find(":") == -1:
			        self.routes.append("."+lane[:-2]+".")
			        traci.route.add(str(lane[:-2]),[str(lane[:-2])])

		def get_path_list(Tree, routes_list):
			path_tree = []
			start_node = Tree.path
			distance = Tree.distance
			for edge in routes_list:
				if edge.find(":") == -1 and edge.find("."+str(start_node[len(start_node)-1])+"to") != -1:
					temp = tree()
					temp.path.extend(start_node)
					temp.path.append(int(edge[len("."+str(start_node[len(start_node)-1])+"to"):-1]))
					temp.distance = (distance - traci.lane.getLength(edge[1:-1]+"_0")/100)
					if temp.distance >= 0:
						temp_tree = get_path_list(temp, routes_list)
						for added_path in temp_tree:
							path_tree.append(added_path)
						path_tree.append(temp.path)
			return path_tree

		def getpaths():
			time_periods = self.node.nodes[:,3]
			max_path = max(time_periods[np.where(self.node.nodes[:,2] == 1)])
			path_exists = os.path.exists("result/paths_list.txt")
			if path_exists == False:
				for node_id in (self.node).interest_nodes:
					temp = tree()
					temp.path.append(node_id)
					temp.distance = max_path
					self.paths = self.paths + (get_path_list(temp,self.routes))
				with open("result/paths_list.txt", "w") as file:
					file.write(str(self.paths))
				for node_i in self.node.interest_nodes:
					for node_j in self.node.interest_nodes:
						for path in self.paths:
							if path[0] == node_i and path[len(path)-1] == node_j and (path_length(path)<self.shortest_path[node_i,node_j]):
								self.shortest_path[node_i,node_j] = int(path_length(path))
				np.savetxt("result/shortest_paths_list.txt", self.shortest_path, fmt='%d')

			else:
				with open("result/paths_list.txt", "r") as file:
					self.paths = json.load(file)
				self.shortest_path = np.loadtxt("result/shortest_paths_list.txt", dtype=int)

		self.getpaths = getpaths

		#Initialization of params
		def setup():
			gen_route()
			#Spawing vehicles at the nodes of interest
			for node in range(len(self.node.nodes)):
				x = traci.junction.getPosition(str(node))[0]
				y = traci.junction.getPosition(str(node))[1]
				if x>self.max_x:
					self.max_x = x
				if y>self.max_y:
					self.max_y = y
				if x<self.min_x:
					self.min_x = x
				if y<self.min_y:
					self.min_y = y
			self.heatmap = np.zeros((int(self.max_x-self.min_x)/self.size+1,int(self.max_y-self.min_y)/self.size+1))
			#For Grid
			""" veh = vehicle.addVehicle("1to0",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			veh = vehicle.addVehicle("23to24",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			veh = vehicle.addVehicle("9to4", (self.node).num_nodes, len(self.vehicles), self.speed,self.node)
			self.vehicles.append(veh) """
			""" veh = vehicle.addVehicle("15to20", (self.node).num_nodes, len(self.vehicles), self.speed,self.node)
			self.vehicles.append(veh) """
			#For Random
			veh = vehicle.addVehicle("6to5",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			veh = vehicle.addVehicle("1to4",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			#FOR CPG
			""" veh = vehicle.addVehicle("1to0",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			veh = vehicle.addVehicle("8to7",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			veh = vehicle.addVehicle("15to10", (self.node).num_nodes, len(self.vehicles), self.speed,self.node)
			self.vehicles.append(veh) """
			#FOR CPR
			""" veh = vehicle.addVehicle("4to1",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh)
			veh = vehicle.addVehicle("2to7",(self.node).num_nodes,len(self.vehicles),self.speed,self.node)
			self.vehicles.append(veh) """
			self.node.setup()
			self.getpaths()
		self.setup = setup

		def step():
			if traci.simulation.getMinExpectedNumber()>0:
				#Increment time
				traci.simulationStep()
				#Populate the list of departed vehicles
				for veh_id in traci.simulation.getDepartedIDList():
					(self.departed).append(veh_id)
				#Update params of the vehicles
				for veh_id in self.departed:
					curr_vehicle = self.vehicles[int(veh_id)]
					curr_vehicle.updateParams()
				#Increase the idleness
				self.node.nodes[:,1] = self.node.nodes[:,1] + 0.1 
				for node_id in self.node.interest_nodes:
					self.node.nodes[node_id,4] = self.node.nodes[node_id,3] - self.node.nodes[node_id,1]
				#Call the algorithm for assigning nodes
				self.node.nodes = patrolling.TPP(self.departed,self.vehicles,self.node,self.routes,self.paths,self.shortest_path)
		self.step = step


def simulate():
	simulator = simulation()
	simulator.setup()
	with open("result/idleness.csv", "w") as file:
		writer = csv.writer(file)
		writer.writerow(['time','node','idleness','t_imp','visits'])
		for i in tqdm(range(simulator.total_time*10)):
			simulator.step()
			simulator.curr_time = simulator.curr_time + 0.1
			for node in range(len(simulator.node.nodes)):
				row = simulator.curr_time,node,simulator.node.nodes[node,1],simulator.node.nodes[node,4],simulator.node.visited[node]
				writer.writerow(row)
	#Create the heatmap
	for node in range(len(simulator.node.nodes)):
		x = int(traci.junction.getPosition(str(node))[0])
		y = int(traci.junction.getPosition(str(node))[1])
		simulator.heatmap[int((x-simulator.min_x)/simulator.size), int((y-simulator.min_y) /
											simulator.size)] = simulator.node.visited[node]
		# if(simulator.node.visited[node] == 0):
		# 	simulator.heatmap[int((x-simulator.min_x)/simulator.size),
		# 								int((y-simulator.min_y)/simulator.size)] = simulator.total_time
	print("Average Idleness",np.mean(simulator.total_time/simulator.node.visited))
	print("Number of Visits")
	print(np.flip(simulator.heatmap.transpose(),0))
	sns.heatmap(simulator.heatmap.transpose(),vmin=0,vmax=max(simulator.node.visited),cbar_kws={'label': '#Visits'}).invert_yaxis()
	plt.xlabel('X Co-ordinate')
	plt.ylabel('Y Co-ordinate')
	plt.savefig("result/heatmap.png")
	plt.show()
	traci.close()
	sys.stdout.flush()
