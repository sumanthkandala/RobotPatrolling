from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import os
import sys
import optparse
import subprocess
import random
import numpy
import math
import statistics

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
max_speed = 10
num_vehicles = 0
num_nodes = 25
idleness = numpy.zeros(shape=(num_nodes,2))
avg_idleness = [0]*num_nodes
global avg_idleness
global idleness
idle_weight = 0.5

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
    #PERFORM SIMULATION
    step = 0
    traci.vehicle.add(str(step),"0to1",speed=0)
    while traci.simulation.getMinExpectedNumber() > 0:
        curr_time = traci.simulation.getCurrentTime()
        traci.simulationStep()

        #ADD DEPARTED VEHICLES TO THE LIST
        for veh_id in traci.simulation.getDepartedIDList():
        	print(num_vehicles, " ",veh_id)
        	departed.append(veh_id)

        #REMOVE FROM DEPARTED
        for veh_id in traci.simulation.getArrivedIDList():
        	departed.remove(veh_id)

        #print(curr_dest


        #PERFORM COMPUTATION FOR EACH DEPARTED VEHICLE

        # USE IF assign_all OR assign_vehicle IS ENABLED
        # if count == 1:
        # 	assign_all()

        step += 1
        traci.vehicle.changeTarget("0","5to0")
        if traci.vehicle.getRoadID("0").find("5to0") != -1:
        	traci.vehicle.changeTarget("0","0to1") 
    ######################################################

    #CLOSE SIMULATION
    traci.close()
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
