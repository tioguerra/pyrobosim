from math import *

class reference_tejectory:
    def __init__(self,apha,delta,omega,sigma,C):
        self.alpha = alpha
        self.delta = delta
        self.omega = omega
        self.sigma = sigma
        self.C = C
    def exchange_state(Vx,Vy,Vphi,lamda):
        norm = sqrt(Vx**2 + Vy**2 + Vphi**2)
        Vx = Vx/norm
        Vy = Vy/norm
        Vphi = Vphi/norm

        ksi = self.delta + abs(Vy)*(omega - delta)
        tau = (1/C) * log((ksi/alpha) + sqrt((ksi**2)/(alpha**2) - 1))

        Sx = Vx*(sigma/C)*sinh(C*tau)
        Sx_vel = Vx*sigma*cosh(C*tau)
        if abs(Vy) == Vy:
            if lamda > 0:
                Sy = lamda * ksi
            else:
                Sy = lamda * delta
        else:
            if lamda < 0:
                Sy = lamda * ksi
            else:
                Sy = lamda * delta
        Sy_vel = lamda*C*sqrt((Sy**2) - (alpha**2))
        return Sx, Sx_vel, Sy, Sy_vel
