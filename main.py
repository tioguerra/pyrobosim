from sim import Sim
from robot import Robot
from controller import *

# Create a simulation
s = Sim()
# Create a robot 50cm above the ground
r = Robot(s, (0.0, 0.5, 0.0))
# Create a controller
c = PeriodCtrl(s,r)
# Start the simulation
s.start()

