import sys
import time
from math import *
from controller import UDPCtrlClient
from control_interface import control_interface
from motion_pattern import motion_pattern
from leg_interface import leg_interface


if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'url','server_port client_port'
    sys.exit(0)

c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

RightTau = 0.0
LeftTau = -pi
TauStep = 2.0*pi / 200.0

control = control_interface([3.5,0.0085,0.01,0.009,0.09,0.008,0])
motion = motion_pattern([0.02,0.1,0.02,0.03,0,0.02,0,0.3,0.12,0.17,0.12,0.1,0.05,0.015,0.2,0.05,0.035,0,0,-0.07],[0,2.3876])
leg = leg_interface()

while True:
    #calculates angles for the left leg
    eta, left_roll_leg, left_pitch_leg, left_yaw_leg,left_roll_foot, left_pitch_foot = motion.final_motion(0.0,0.5,0.0,RightTau,-1)
    left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle_pitch, left_ankle_roll = \
                              leg.joint_angles(eta, left_roll_leg, left_pitch_leg, left_yaw_leg, left_roll_foot, left_pitch_foot)
    #calculates angles for the right leg
    eta, right_roll_leg, right_pitch_leg, right_yaw_leg,right_roll_foot, right_pitch_foot = motion.final_motion(0.0,0.5,0.0,LeftTau,1)
    right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle_pitch, right_ankle_roll = \
                              leg.joint_angles(eta, right_roll_leg, right_pitch_leg, right_yaw_leg, right_roll_foot, right_pitch_foot)

    #sends the desired angles to server 
    
    command = [left_hip_roll[0],right_hip_roll[0],left_hip_pitch[0],right_hip_pitch[0],left_hip_yaw,right_hip_yaw,\
            -left_knee,-right_knee,left_ankle_roll[0],right_ankle_roll[0],left_ankle_pitch[0],right_ankle_pitch[0]]

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

