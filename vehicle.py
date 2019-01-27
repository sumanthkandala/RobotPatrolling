from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import os
import sys
import optparse
import subprocess
import numpy
import math

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

class veh:
	def __init__(self,curr_edge,num_nodes,nodes):
		self.veh_id = 0	#Vehicle ID
		self.steps = [0]*num_nodes	#Distance to the node
		self.cost = [0]*num_nodes	#Cost of the node
		self.start_node = 0	#Node just departed
		self.end_node = 1	#Node about to reach
		self.dest_node = 1	#Node planned to reach
		self.dest_edge = curr_edge 	#Edge planned to reach
		self.curr_edge = curr_edge  #Edge presently on
		self.origin = 0 #Origin Node (For DTAG and for CPG)
		self.vehicles = 0
		self.path = []
		self.path.append(int(curr_edge[:curr_edge.find("to")]))
		self.path.append(int(curr_edge[curr_edge.find("to")+2:]))

		#FUNCTION TO CHANGE THE TARGET OF THE VEHICLE
		def changeTarget(edge):
			traci.vehicle.changeTarget(str(self.veh_id),edge)
			self.dest_edge = edge
		self.changeTarget = changeTarget

		####################################################

		#UPDATE CURR_EDGE AND CURRENT DESTINATION AND SET IDLENESS OF VISITED NODE
		def updateParams():
			temp = traci.vehicle.getRoadID(str(self.veh_id))
			if self.curr_edge != temp and temp.find(":") == -1 and len(temp)>1:
				nodes.visited[int(self.curr_edge[self.curr_edge.find("to")+2:])] = nodes.visited[int(self.curr_edge[self.curr_edge.find("to")+2:])] + 1 
				nodes.nodes[int(self.curr_edge[self.curr_edge.find("to")+2:]),1] = 0 
				self.curr_edge = temp
		self.updateParams = updateParams

		####################################################


def addVehicle(start_edge,num_nodes,num_vehicles,max_speed,nodes):
	traci.vehicle.add(str(num_vehicles),start_edge,speed=0)
	traci.vehicle.setMaxSpeed(str(num_vehicles),max_speed)
	vehicle = veh(start_edge,num_nodes,nodes)
	vehicle.dest_node = int(start_edge[start_edge.find("to")+2:])
	vehicle.veh_id = str(num_vehicles)
	vehicle.origin = int(start_edge[:start_edge.find("to")])
	return vehicle
