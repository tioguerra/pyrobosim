from math import *
from numpy import *

class control_interface:
    def __init__(self,C):
        self.Vx = 0
        self.Vy = 0
        self.Vphi = 0
        self.tau = 0
        self.Vx_max = 0
        self.Vy_max = 0
        self.Vphi_max = 0
        self.C = []
        for i in range(len(C)):
            self.C.append(C[i])

    def setVelocity(self,Vx,Vy,Vphi):
        self.Vx_max, self.Vy_max, self.Vphi_max = self.velocity_restriction(Vx,Vy,Vphi)

    def p_norm(self,Vx,Vy,Vphi):
        V_norm = (Vx**self.C[0] + Vy**self.C[0] + Vphi**self.C[0])**(1/self.C[0])
        return V_norm

    def velocity_restriction(self,Vx,Vy,Vphi):
        V_norm = self.p_norm(Vx,Vy,Vphi)
        if V_norm > 1:
            Vx = Vx/V_norm
            Vy = Vy/V_norm
            Vphi = Vphi/V_norm
        else:
            pass
        return Vx,Vy,Vphi
    
    def velocity_incrementation(self):
        self.Vx = self.Vx + max(-self.C[1],min(self.Vx_max - self.Vx, self.C[1]))
        self.Vy = self.Vy + max(-self.C[2],min(self.Vy_max - self.Vy, self.C[2]))
        self.Vphi = self.Vphi + max(-self.C[3],min(self.Vphi_max - self.Vphi, self.C[3]))

    def motion_phase(self):
        self.tau = self.tau + self.C[4] + abs(self.Vx)*self.C[5] + abs(self.Vy)*self.C[6]

        if self.tau > pi:
            self.tau = self.tau - (2*pi)
        else:
            pass
        return self.tau

    def full_controll(self,Vx,Vy,Vphi):
        self.setVelocity(Vx,Vy,Vphi)
        self.velocity_incrementation()
        self.motion_phase()
        return self.Vx, self.Vy, self.Vphi, self.tau
