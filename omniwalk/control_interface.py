from math import *
from numpy import *
from constants import *

class ControlInterface:
    def __init__(self):
        self.Vx = 0
        self.Vy = 0
        self.Vphi = 0
        self.tau = 0
        self.Vx_max = 0
        self.Vy_max = 0
        self.Vphi_max = 0
        self.start = 0

    def setVelocity(self,Vx,Vy,Vphi):
        self.Vx_max, self.Vy_max, self.Vphi_max = self.velocity_restriction(Vx,Vy,Vphi)

    def p_norm(self,Vx,Vy,Vphi):
        V_norm = (abs(Vx)**C21 + abs(Vy)**C21 + abs(Vphi)**C21)**(1/C21)
        return V_norm

    def velocity_restriction(self,Vx,Vy,Vphi):
        V_norm = self.p_norm(Vx,Vy,Vphi)
        if V_norm > 1:
            Vx = Vx/V_norm
            Vy = Vy/V_norm
            Vphi = Vphi/V_norm
        return Vx,Vy,Vphi
    
    def velocity_increment(self):
        self.Vx = self.Vx + max(-C22,min(self.Vx_max - self.Vx, C22))
        self.Vy = self.Vy + max(-C23,min(self.Vy_max - self.Vy, C23))
        self.Vphi = self.Vphi + max(-C24,min(self.Vphi_max - self.Vphi, C24))

    def motion_phase(self):
        self.tau = self.tau + C25 + abs(self.Vx)*C26 + abs(self.Vy)*C27

        if self.tau > pi:
            self.tau = self.tau - (2*pi)
        return self.tau

    def start_increment(self):
        if self.start <1.0:
            self.start = self.start + C28

    def full_control(self,Vx,Vy,Vphi):
        self.setVelocity(Vx,Vy,Vphi)
        self.velocity_increment()
        self.motion_phase()
        self.start_increment()

        return self.Vx, self.Vy, self.Vphi, self.tau, self.start
