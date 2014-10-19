import sys
from sim import Sim
from robot import BipedRobot, QuadrupedRobot
from controller import *

if len(sys.argv) != 2:
    print 'Usage:', sys.argv[0],'server_port'
    sys.exit(0)

# Create a simulation
s = Sim()

# Create a robot 50cm above the ground
# obs.: changing last argument to False
#       allows it to fall
<<<<<<< HEAD:server.py
r = BipedRobot(s, (0.0, 0.0, 0.0), False)
#r.setTargets({'lAnkleRoll': -0.07, 'rKnee': -0.40066968464623937, 'rAnkleTilt':0.23033484232311968, 'lAnkleTilt': 0.23033484232311968, 'lHipRoll': -0.1,'rHipRoll': 0.1, 'lHipTilt': 0.18033484232311969, 'lKnee':-0.40066968464623937, 'rHipYaw': 0.0, 'lHipYaw': 0.0, 'rAnkleRoll': -0.07,'rHipTilt': 0.18033484232311969})
=======
r = QuadrupedRobot(s, (0.0, 0.30, 0.0), False)

>>>>>>> ad162210184e5f253cf0ae75f42b5b261f5806c8:quadruped_server.py
# Create a controller
c = UDPCtrlServer(s, r, sys.argv[1])

# Start the simulation
s.start()

