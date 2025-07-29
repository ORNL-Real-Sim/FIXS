import os, sys, socket, re, json, random
import numpy as np

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

#sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
#sumoCmd = [sumoBinary, "-c", "longhighway.sumo.cfg", "--start"]

import traci
import sumolib
from sumolib import checkBinary
import sumolib.net
from sumolib.net import readNet
from sumolib.net import Net
from sumolib.net import NetReader
from sumolib.net import lane
from sumolib.net import edge
from sumolib.net import node 
from sumolib.net import connection
from sumolib.net import roundabout
from sumolib.net.edge import Edge

def main():

    v0 = 30/1.6
    dis = h*v0 + 4

    graph = sumolib.net.readNet('./environment.net.xml', withInternal=True) #internal edge are edges inside interseciton or connections 
    vertex = graph.getNodes()
    edge = graph.getEdges(withInternal=True)
    #print('edge length:')
    #print(len(edge))

    sumoCmd = [sumolib.checkBinary('sumo-gui'), '-c', './Intersection.sumo.cfg']
    traci.start(sumoCmd)
    step = 0
    maxIter = 16000

    traci.route.add(routeID = 'route1', edges = ['E2TL', 'TL2W'])
    traci.vehicle.add('ego', 'route1', departPos=str(1000), departSpeed=str(v0), departLane = str(1), typeID="passenger")
    traci.vehicle.setColor('ego', color=(255, 0, 0, 255))
    traci.vehicle.setLaneChangeMode('ego', 256)

    trigger_dis = 60
    triggered = 0

    traci.gui.trackVehicle("View #0", 'ego')
    traci.gui.setZoom("View #0", 1500)
    pos_record = []

    while step < maxIter:
        traci.simulationStep()
        
        eEdge = traci.vehicle.getRoadID('ego')
        ePos = traci.vehicle.getPosition('ego')
        eLanePos = traci.vehicle.getLanePosition('ego')
        traci.vehicle.setSpeed('ego', v0)
        
        if not triggered and (1750 - eLanePos >= trigger_dis-0.2 and 1750 - eLanePos <= trigger_dis+0.2):
            triggered = 1
            traci.vehicle.add('pred', 'route1', departPos=str(1750-20), departSpeed=str(5), departLane = str(1), typeID="passenger")
            traci.vehicle.setColor('pred', color=(255, 255, 0, 255))
            traci.vehicle.setLaneChangeMode('pred', 256)
            trigger_step = step

        if triggered:
            if step > trigger_step + 12/0.01:
                traci.trafficlight.setRedYellowGreenState('TL', "rrrrrGGGGrrrrrrGGGGr")
            #traci.vehicle.moveTo('pred', str(1), pos_toMove)
            traci.vehicle.setSpeed('pred', v0)
        else:
            traci.trafficlight.setRedYellowGreenState('TL', "GGGGrrrrrrGGGGrrrrrr")
        
        traci.vehicle.setSpeed('ego', v0)
            
        pos_record.append(eLanePos)
        
        step += 1

    traci.close()


if __name__ == "__main__":
    main()