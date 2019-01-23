from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import os
import sys
import optparse
import subprocess
import numpy as np
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

class node:
	def __init__(self):
		self.num_nodes = 25
		self.nodes = np.zeros((self.num_nodes,6))
		self.interest_nodes = [0,4,20,24]
		self.targets = []
		self.visited = np.zeros(self.num_nodes)

		def setup():
			for node_id in range(self.num_nodes):
				self.nodes[node_id][0] = node_id
			for node_id in self.interest_nodes:
				self.nodes[node_id][1] = 0 #Idleness
				self.nodes[node_id][2] = 1 #1 indicates a node of interest
				self.nodes[node_id][3] = 4 #Timeperiod
				self.nodes[node_id][4] = 0 #T_importance
				self.nodes[node_id][5] = 1 #1 == Unvisited
			#self.nodes[6][3] = 41
		self.setup = setup
