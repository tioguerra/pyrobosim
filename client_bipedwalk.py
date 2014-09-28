import sys
import time
from math import *
from controller import UDPCtrlClient
from omniwalk import *


if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'url','server_port client_port'
    sys.exit(0)

c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

RightTau = 0.0
LeftTau = -pi
TauStep = 2.0*pi / 200.0

control = ControlInterface([3.5,0.0085,0.01,0.009,0.09,0.008,0])
motion = MotionPattern()
leg = LegInterface()

while True:
    #calculates angles for the left leg
    angles = motion.final_motion(0.5,0.0,0.0,RightTau,-1)
    rightJointAngles = leg.joint_angles(angles)
    #calculates angles for the right leg
    angles = motion.final_motion(0.5,0.0,0.0,LeftTau,1)
    leftJointAngles = leg.joint_angles(angles)

    #sends the desired angles to server 
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

    r = c.sendCommand(command)
    print r

    #increment and adjusts Tau
    RightTau = RightTau + TauStep
    LeftTau = LeftTau + TauStep
    if RightTau > pi:
        RightTau = RightTau - 2*pi
    if LeftTau > pi:
        LeftTau = LeftTau - 2*pi
    time.sleep(0.01)

while True:
    for command in commands:
        r = c.sendCommand(command)
        print r
        time.sleep(1)

