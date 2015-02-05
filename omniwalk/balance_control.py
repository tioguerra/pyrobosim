from math import *
from constants import *

class balance_control:
    def __init__(self):
        self.Zy = 0
        self.Zx = 0
        self.T = 0
        def balance(self,Sx,Sx_dot,Sy,Sy_dot,cx,cx_dot,cy,cy_dot):
            #lateral ZMP calculation
            self.Zy = (Sy*2*C*exp(C*T_ideal) - cy*C*(1+exp(2*C*T_ideal)) + cy_dot*(1-exp(2*C*T_ideal)))/ \
                    C*(exp(2*C*T_ideal) - 2*exp(C*T_ideal) + 1)
            #step time prediction
            self.T = 1/C * ln( (Sy - Zy)/(cy - Zy + (cy_dot/C)) + sqrt( ((Sy - Zy)**2)/(cy - Zy + (cy_dot/C))**2 \
                    - ((cy - Zy - (cy_dot/C))/(cy - Zy + (cy_dot/C)))))
            #saggital ZMP calculation
            self.Zx = Sx + (Sx_dot/C) - exp(C*self.T)*(cx + (cx_dot/C))
            return

            #end of step states
        def endStep(self,cx,cx_dot,cy,cy_dot):
            #Cx = (cx - self.Zx) * cosh(C*self.T) + (cx_dot/C)*sinh(C*self.T)
            Cx_dot = (cx - self.Zx) * C*sinh(C*self.T) + cx_dot*cosh(C*self.T)
            #Cy = (cy - self.Zy) * cosh(C*self.T) + (cy_dot/C)*sinh(C*self.T)
            Cy_dot = (cy - self.Zy) * C*sinh(C*self.T) + cy_dot*cosh(C*self.T)

            return Cx,Cx_dot,Cy,Cy_dot

            #new position for the support foot
        def footstep_location(self,Cx_dot,Cy_dot,tau):
            F = ((Cx_dot/C)*tanh(C*tau), lamda*sqrt(((Cy_dot**2)/(C**2) + alpha**2)))
            return F

        def full_balance(self):
            balance(Sx,Sx_dot,Sy,Sy_dot,cx,cx_dot,cy,cy_dot)
            endStep(cx,cx_dot,cy,cy_dot)
            footstep_location(Cx_dot,Cy_dot)




