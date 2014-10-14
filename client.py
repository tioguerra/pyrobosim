import sys
import time
from math import *
from controller import UDPCtrlClient

if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'url','server_port client_port'
    sys.exit(0)

c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

commands = [\
        {'flHipRoll': pi/4, 'brHipRoll': pi/4},\
                                              {'flKnee':    pi/4, 'brKnee':    pi/4},\
        {'flHipRoll':    0, 'brHipRoll':    0, 'flKnee':       0, 'brKnee':       0},\
        {'frHipRoll': pi/4, 'blHipRoll': pi/4},\
                                              {'frKnee':    pi/4, 'blKnee':    pi/4},\
        {'frHipRoll':    0, 'blHipRoll':    0,   'frKnee':     0, 'blKnee':       0},\
        {'frHipYaw':  pi/6, 'brHipYaw': -pi/6, 'flHipYaw':  pi/6, 'blHipYaw': -pi/6},\
        {'frHipYaw':     0, 'brHipYaw':     0, 'flHipYaw':     0, 'blHipYaw':     0},\
        {'frHipYaw': -pi/6, 'brHipYaw':  pi/6, 'flHipYaw': -pi/6, 'blHipYaw':  pi/6},\
        {'frHipYaw':     0, 'brHipYaw':     0, 'flHipYaw':     0, 'blHipYaw':     0},\
        ]

while True:
    for command in commands:
        r = c.sendCommand(command)
        print r
        time.sleep(1)

