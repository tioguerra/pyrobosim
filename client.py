import sys
import time
from math import *
from controller import UDPCtrlClient

if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'url','server_port client_port'
    sys.exit(0)

c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

commands = [[   0.0 ,  00.0 ,  0.0 , pi/4 ,0,0,  0.0  , -pi/4 ,0,0,0,0],
            [   0.0 ,  00.0 , pi/4 ,  0.0 ,0,0, -pi/4 ,  0.0  ,0,0,0,0],
            [   0.0 ,  00.0 ,  0.0 ,  0.0 ,0,0,  0.0  ,  0.0  ,0,0,0,0],
            [  pi/6 ,  pi/6 ,  0.0 ,  0.0 ,0,0,  0.0  ,  0.0  ,0,0,0,0]]

while True:
    for command in commands:
        r = c.sendCommand(command)
        print r
        time.sleep(1)

