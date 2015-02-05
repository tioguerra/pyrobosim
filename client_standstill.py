import sys
import time
from math import *
from controller import UDPCtrlClient

if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'url','server_port client_port'
    sys.exit(0)

c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

initialTargets = {'lAnkleRoll': 0.0, 'rKnee': 0.0, 'rAnkleTilt':0.0, 'lAnkleTilt': 0.0, 'lHipRoll': 0.0,'rHipRoll': 0.1, 'lHipTilt': 0.18033484232311969, 'lKnee':0.0, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': 0.0,'rHipTilt': 0.0}

finalTargets = {'lAnkleRoll': -0.07002828646836044, 'rKnee': -0.49131053823344362, 'rAnkleTilt': 0.29065526911672179, 'lAnkleTilt': 0.29055654058733033, 'lHipRoll': 0.10002828646836044, 'rHipRoll': 0.09997171353163957, 'lHipTilt': 0.19055654058733035, 'lKnee': -0.49111308117466068, 'rHipYaw': -0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.06997171353163957, 'rHipTilt': 0.19065526911672181}

steps = 250

for step in range(steps):
    targets = {}
    for key in initialTargets:
        targets[key] = initialTargets[key] + (step+1)*(finalTargets[key] - initialTargets[key]) / float(steps)
    c.sendCommand(targets)
    time.sleep(0.01)

