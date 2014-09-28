from math import *
from numpy import *

class LegInterface:
    def __init__(self):
        pass
    def joint_angles(self, angles): #eta, theta_leg_roll, theta_leg_pitch, theta_leg_yaw,theta_foot_roll, theta_foot_pitch):
        a = cos(-angles['legYaw'])
        b = -sin(-angles['legYaw'])
        c = sin(-angles['legYaw'])
        d = cos(-angles['legYaw'])
        R = array([[a, b], [c, d]])
        theta = array([[angles['legPitch']],[angles['legRoll']]])
        theta_prime = dot(R,theta)

        lamda = arccos(1.0 - angles['eta'])
        jointAngles = {'hipYaw':angles['legYaw'],\
                       'hipRoll':theta_prime[0][0],\
                       'hipPitch':theta_prime[1][0]-lamda,\
                       'knee':2*lamda,\
                       'anklePitch':angles['footPitch']-theta_prime[0][0]-lamda,\
                       'ankleRoll':angles['footRoll']-theta_prime[1][0]}
        return jointAngles

