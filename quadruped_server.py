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

r = QuadrupedRobot(s, (0.0, 0.60, 0.0), False)

# Create a controller
c = UDPCtrlServer(s, r, sys.argv[1])

# Start the simulation
s.start()

