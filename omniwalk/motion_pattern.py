from constants import *
from math import *

class MotionPattern:
    def __init__(self):
        pass
    def halt_position(self,sigma):
        P = {'etaHalt':C1,\
             'legRollHalt':sigma*C2,\
             'legPitchHalt':C3,\
             'footRollHalt':C4,\
             'footPitchHalt':C5}
        return P
    def leg_lifting(self,tau,Vx,Vy):
        P = {}
        if tau <= 0:
            P['legLift'] = sin(tau) * (C6 + C7 * max((abs(Vx),abs(Vy))))
        elif tau > 0:
            P['legLift'] = sin(tau) * (C8 + C9 * max((abs(Vx),abs(Vy))))
        return P
    def leg_swing(self,tau,Vx,Vy,Vphi,sigma):
        if C_tau_0 <= tau and C_tau_1 > tau:
            gamma = cos(((tau-C_tau_0)/(C_tau_1-C_tau_0))*pi)
        elif C_tau_1 <= tau and tau < pi:
            gamma = 2*(tau-C_tau_1)/(2*pi-C_tau_1+C_tau_0)-1
        elif tau >= -pi and tau < C_tau_0:
            gamma = 2*(tau+2*pi-C_tau_1)/(2*pi-C_tau_1+C_tau_0)-1
        P = {}
        if Vx >= 0:
            P['pitchLegSwing'] = gamma*Vx*C10
        elif Vx < 0:
            P['pitchLegSwing'] = gamma*Vx*C11
        P['rollLegSwing'] = -gamma*Vy*C12 - sigma*max(C13*abs(Vx),C14*abs(Vphi))
        P['yawLegSwing'] = gamma*Vphi*C15 - sigma*C16*abs(Vphi)
        return P
    def hip_swing(self,tau):
        if tau < C_tau_0:
            tau_l = tau-C_tau_1+2*pi
        elif tau > C_tau_1:
            tau_l = tau-C_tau_1
        else:
            tau_l = 0.0
        if (tau+pi) < C_tau_0:
            tau_r = tau-C_tau_1+3*pi
        elif (tau+pi) > C_tau_1:
            tau_r = tau-C_tau_1+pi
        else:
            tau_r = 0.0
        delta = C_tau_0 - C_tau_1 + 2*pi
        P = {'hipSwing':C17*(sin(tau_l*pi/delta)-sin(tau_r*pi/delta))}
        return P
    def leaning(self,Vx,Vphi):
        P = {}
        if Vx >= 0:
            P['pitchLean'] = Vx*C18
        elif Vx < 0:
            P['pitchLean'] = Vx*C19
        P['rollLean'] = -Vphi*abs(Vx)*C20
        return P
    def final_motion(self,Vx,Vy,Vphi,tau,sigma):
        P = {}
        P.update(self.halt_position(sigma))
        P.update(self.leg_lifting(tau,Vx,Vy))
        P.update(self.leg_swing(tau,Vx,Vy,Vphi,sigma))
        P.update(self.hip_swing(tau))
        P.update(self.leaning(Vx,Vphi))
        angles = {'eta':P['etaHalt']+P['legLift'],\
                  'legRoll':P['legRollHalt']+P['hipSwing']+P['rollLegSwing']+P['rollLean'],\
                  'legPitch':P['legPitchHalt']+P['pitchLegSwing']+P['pitchLean'],\
                  'legYaw':P['yawLegSwing'],\
                  'footRoll':P['footRollHalt'],\
                  'footPitch':P['footPitchHalt']}
        return angles

