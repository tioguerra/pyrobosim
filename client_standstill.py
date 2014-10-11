import sys
import time
from math import *
from controller import UDPCtrlClient

if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'url','server_port client_port'
    sys.exit(0)

c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

initialTargets = {'lAnkleRoll': 0.0, 'rKnee': 0.0, 'rAnkleTilt':0.0, 'lAnkleTilt': 0.0, 'lHipRoll': 0.0,'rHipRoll': 0.1, 'lHipTilt': 0.18033484232311969, 'lKnee':0.0, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': 0.0,'rHipTilt': 0.0}

#finalTargets = {'lAnkleRoll': -0.07, 'rKnee': -0.40066968464623937, 'rAnkleTilt':0.23033484232311968, 'lAnkleTilt': 0.23033484232311968, 'lHipRoll': -0.1,'rHipRoll': 0.1, 'lHipTilt': 0.18033484232311969, 'lKnee':-0.40066968464623937, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.07,'rHipTilt': 0.18033484232311969}
#finalTargets = {'lAnkleRoll': -0.03996001083571739, 'rKnee': -0.40066968464623937, 'rAnkleTilt': 0.2453348423231197, 'lAnkleTilt': 0.2453348423231197, 'lHipRoll': 0.06996001083571739, 'rHipRoll': 0.06996001083571739, 'lHipTilt': 0.18033484232311969, 'lKnee': -0.40066968464623937, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.03996001083571739, 'rHipTilt': 0.18033484232311969}
#finalTargets = {'lAnkleRoll': -0.03714363112267023, 'rKnee': -0.40066968464623937, 'rAnkleTilt': 0.2453348423231197, 'lAnkleTilt': 0.2453348423231197, 'lHipRoll': 0.06714363112267023, 'rHipRoll': 0.06714363112267023, 'lHipTilt': 0.18033484232311969, 'lKnee': -0.40066968464623937, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.03714363112267023, 'rHipTilt': 0.18033484232311969}
finalTargets = {'lAnkleRoll': -0.04003067464806073, 'rKnee': -0.49118446938974419, 'rAnkleTilt': 0.26559223469487209, 'lAnkleTilt': 0.26556391439394639, 'lHipRoll': 0.07003067464806073, 'rHipRoll': 0.06996932535193928, 'lHipTilt': 0.22556391439394641, 'lKnee': -0.4911278287878928, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.03996932535193928, 'rHipTilt': 0.22559223469487211}
#finalTargets = {'lAnkleRoll': -0.029969007926221113, 'rKnee': -0.491192675166647, 'rAnkleTilt': 0.2455963375833235, 'lAnkleTilt': 0.24556366819068598, 'lHipRoll': 0.05996900792622111, 'rHipRoll': 0.05996900792622111, 'lHipTilt': 0.24556366819068598, 'lKnee': -0.49112733638137196, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.029969007926221113, 'rHipTilt': 0.2455963375833235}
steps = 250

for step in range(steps):
    targets = {}
    for key in initialTargets:
        targets[key] = initialTargets[key] + (step+1)*(finalTargets[key] - initialTargets[key]) / float(steps)
    c.sendCommand(targets)
    time.sleep(0.01)

