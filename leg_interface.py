from math import *
from numpy import *

class leg_interface:
    def __init__(self):
        pass
    def joint_angles(self, eta, theta_leg_roll, theta_leg_pitch, theta_leg_yaw,theta_foot_roll, theta_foot_pitch):
        a = cos(-theta_leg_yaw)
        b = -sin(-theta_leg_yaw)
        c = sin(-theta_leg_yaw)
        d = cos(-theta_leg_yaw)
        R = array([[a, b], [c, d]])
        theta = array([[theta_leg_pitch],[theta_leg_roll]])
        theta_prime = dot(R,theta)

        lamda = arccos(1 - eta)
        theta_hip_yaw = theta_leg_yaw
        theta_hip_roll = theta_prime[0]
        theta_hip_pitch = theta_prime[1] - lamda
        theta_knee = 2*lamda
        theta_ankle_pitch = theta_foot_pitch - theta_prime[0] - lamda
        theta_ankle_roll = theta_foot_roll - theta_prime[1]
        return theta_hip_yaw, theta_hip_roll, theta_hip_pitch, theta_knee, theta_ankle_pitch, theta_ankle_roll
