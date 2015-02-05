from math import *
from constants import *

class reference_trajectory:
    def __init__(self):
        pass
    def exchange_state(Vx,Vy,Vphi,lamda):
        norm = sqrt(Vx**2 + Vy**2 + Vphi**2)
        Vx = Vx/norm
        Vy = Vy/norm
        Vphi = Vphi/norm

        ksi = delta + abs(Vy)*(omega - delta)
        tau = (1/C) * log((ksi/alpha) + sqrt((ksi**2)/(alpha**2) - 1))

        #definition of a target state
        Sx = Vx*(sigma/C)*sinh(C*tau)
        Sx_dot = Vx*sigma*cosh(C*tau)
        
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

        Sy_dot = lamda*C*sqrt((Sy**2) - (alpha**2))
        return Sx, Sx_dot, Sy, Sy_dot
