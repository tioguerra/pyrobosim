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
r = QuadrupedRobot(s, (0.0, 1.75, 0.0))
# Create a controller
c = UDPCtrlServer(s, r, sys.argv[1])
# Start the simulation
s.start()

