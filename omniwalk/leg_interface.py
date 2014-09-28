from math import *
from numpy import *

class LegInterface:
    def __init__(self):
        pass
    def joint_angles(self, angles):
        legYaw = angles['legYaw']
        a = cos(-legYaw)
        b = -sin(-legYaw)
        c = -b
        d = a
        R = array([[a, b],\
                   [c, d]])
        theta = array([[angles['legPitch']],\
                       [angles['legRoll']]])
        pitch_leg, roll_leg = dot(R,theta).transpose()[0].tolist()
        lamda = arccos(1.0 - angles['eta'])
        jointAngles = {'hipYaw':legYaw,\
                       'hipRoll':roll_leg,\
                       'hipPitch':pitch_leg-lamda,\
                       'knee':2*lamda,\
                       'anklePitch':angles['footPitch']-pitch_leg-lamda,\
                       'ankleRoll':angles['footRoll']-roll_leg}
        return jointAngles

