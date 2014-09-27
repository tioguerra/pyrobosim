from math import *

class motion_pattern:
    def __init__(self,C,C_tau):

        self.C = []
        self.C_tau = []
        for i in range(len(C)):
            self.C.append(C[i])

        for i in range(len(C_tau)):
            self.C_tau.append(C_tau[i])

        ##-------------Halt Position-------------
    
    def halt_position(self,sigma):
        P_eta_halt = self.C[0]
        P_LRoll_halt = sigma * self.C[1]
        P_LPitch_halt = self.C[2]
        P_FRoll_halt = self.C[3]
        P_FPitch_halt = self.C[4]
        return P_eta_halt, P_LRoll_halt, P_LPitch_halt, P_FRoll_halt, P_FPitch_halt

        ##-------------Leg Liftning--------------

    def leg_lifting(self,tau,Vx,Vy):
        if tau <= 0:
            P_Leg_lift = sin(tau) * (self.C[5] + self.C[6] * max((abs(Vx),abs(Vy))))
        elif tau > 0:
            P_Leg_lift = sin(tau) * (self.C[7] + self.C[8] * max((abs(Vx),abs(Vy))))
        return P_Leg_lift

        ##-------------Leg Swing-----------------

    def Leg_swing(self,tau,Vx,Vy,Vphi,sigma):
        if self.C_tau[0] <= tau and self.C_tau[1] > tau:
            gamma = cos((tau - self.C_tau[0])/(self.C_tau[1]-self.C_tau[0])*pi)
        elif self.C_tau[1] <= tau and tau < pi:
            gamma = 2*(tau - self.C_tau[1])/(2*pi - self.C_tau[1] + self.C_tau[0]) - 1
        elif tau >= -pi and tau < self.C_tau[0]:
            gamma = 2*(tau + 2*pi - self.C_tau[1])/(2*pi - self.C_tau[1] + self.C_tau[0]) - 1
        if Vx >= 0:
            P_pitch_LSwing = gamma*Vx*self.C[9]
        elif Vx < 0:
            P_pitch_LSwing = gamma*Vx*self.C[10]
        P_roll_LSwing = -gamma*Vy*self.C[11] - sigma*max(self.C[12]*abs(Vx),self.C[13]*abs(Vphi))
        P_yaw_LSwing = gamma*Vphi*self.C[14] - sigma*self.C[15]*abs(Vphi)
        return P_pitch_LSwing, P_roll_LSwing, P_yaw_LSwing

        ##------------Lateral Hip Swing---------

    def Hip_swing(self,tau):
        if tau < self.C_tau[0]:
            tau_l = tau - self.C_tau[1] + 2*pi
        elif tau > self.C_tau[1]:
            tau_l = tau - self.C_tau[1]
        else:
            tau_l = 0

        if (tau+pi) < self.C_tau[0]:
            tau_r = tau - self.C_tau[1] + 3*pi
        elif (tau+pi) > self.C_tau[1]:
            tau_r = tau - self.C_tau[1] + pi
        else:
            tau_r = 0
        delta = self.C_tau[0] - self.C_tau[1] + 2*pi
        P_hip_swing = self.C[16] * (sin(tau_l * pi/delta) - sin(tau_r * pi/delta))
        return P_hip_swing

        ##----------------Leaning--------------

    def Leaning(self,Vx,Vphi):
        if Vx >= 0:
            P_pitch_lean = Vx*self.C[17]
        elif Vx < 0:
            P_pitch_lean = Vx*self.C[18]
        P_roll_lean = - Vphi * abs(Vx) * self.C[19]
        return P_pitch_lean, P_roll_lean

        ##--------------Final Motion-----------

    def final_motion(self,Vx,Vy,Vphi,tau,sigma):
        P_eta_halt, P_LRoll_halt, P_LPitch_halt, P_FRoll_halt, P_FPitch_halt = self.halt_position(sigma)
        P_Leg_lift = self.leg_lifting(tau,Vx,Vy)
        P_pitch_LSwing, P_roll_LSwing, P_yaw_LSwing = self.Leg_swing(tau,Vx,Vy,Vphi,sigma)
        P_hip_swing = self.Hip_swing(tau)
        P_pitch_lean, P_roll_lean = self.Leaning(Vx,Vphi)

        eta = P_eta_halt + P_Leg_lift
        theta_roll_leg = P_LRoll_halt + P_hip_swing + P_roll_LSwing + P_roll_lean
        theta_pitch_leg = P_LPitch_halt + P_pitch_LSwing + P_pitch_lean
        theta_yaw_leg = P_yaw_LSwing
        theta_roll_foot = P_FRoll_halt
        theta_pitch_foot = P_FPitch_halt
        return eta, theta_roll_leg, theta_pitch_leg, theta_yaw_leg, theta_roll_foot, theta_pitch_foot
