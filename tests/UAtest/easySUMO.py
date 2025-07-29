import os
import sys

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    raise ImportError("Please declare the environment variable 'SUMO_HOME'")

import numpy as np
import sumolib
import traci

def main():

    sumo_cmd = ['sumo-gui', '-c', 'runmine.sumocfg']
    traci.start(sumo_cmd)
    step = 0

    while step < 36000:
        traci.simulationStep()
        step += 1

    traci.close()


if __name__ == "__main__":
    main()