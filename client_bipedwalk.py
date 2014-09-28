import sys
import time
from math import *
from controller import UDPCtrlClient
from omniwalk import *

# checks for command line arguments
if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'server_url server_port client_port'
    sys.exit(0)

# creates the UDP client interface
c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

# Walking phase
tau = 0.0
tauInc = 2.0*pi / 200.0

# control = ControlInterface([3.5,0.0085,0.01,0.009,0.09,0.008,0])
motion = MotionPattern()
leg = LegInterface()

while True:
    # calculates angles for the left leg
    angles = motion.final_motion(0.5, 0.0, 0.0, tau, -1)
    rightJointAngles = leg.joint_angles(angles)
    # calculates angles for the right leg
    angles = motion.final_motion(0.5, 0.0, 0.0, (tau+2*pi) % (2*pi) - pi, 1)
    leftJointAngles = leg.joint_angles(angles)

    # prepare the omniwalk commands to be sent
    # to the server
    command = {'lHipRoll':leftJointAngles['hipRoll'],\
               'rHipRoll':rightJointAngles['hipRoll'],\
               'lHipTilt':-leftJointAngles['hipPitch'],\
               'rHipTilt':-rightJointAngles['hipPitch'],\
               'lHipYaw':leftJointAngles['hipYaw'],\
               'rHipYaw':rightJointAngles['hipYaw'],\
               'lKnee':-leftJointAngles['knee'],\
               'rKnee':-rightJointAngles['knee'],\
               'lAnkleRoll':leftJointAngles['ankleRoll'],\
               'rAnkleRoll':rightJointAngles['ankleRoll'],\
               'lAnkleTilt':-leftJointAngles['anklePitch'],\
               'rAnkleTilt':-rightJointAngles['anklePitch'],\
               }

    # sends the desired target angles to server, and
    # receive back in r all the current sensor readings
    r = c.sendCommand(command)

    # prints the tilt sensor reading
    if r is not None and 'tilt' in r:
        print 'Tilt sensor reading: %+.3f' % r['tilt']

    # increments tau keeping it in the [-pi,pi) interval
    tau = (tau+tauInc+pi) % (2*pi) - pi
    time.sleep(0.01)

