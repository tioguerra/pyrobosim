from math import *
from numpy import *
import dims

class StateEstimation:
    def __init__(self):
        pass
    def legLength(self, r):
        leg = {}
        #calculates length of each leg
        if r is not None:
            leg['leftEta'] = sqrt(ULEG_LY**2 + LLEG_LY**2 - 2*ULEG_LY*LLEG_LY*cos(pi - r['lKnee']))
            leg['rightEta'] = sqrt(ULEG_LY**2 + LLEG_LY**2 - 2*ULEG_LY*LLEG_LY*cos(pi - r['rKnee']))
            #measures the angle of each leg
            leg['leftLegAngle'] = -arcsin(LLEG_LY*sin(pi - r['lKnee'])/leg['leftEta']) + r['lHipTilt']
            leg['rightLegAngle'] = -arcsin(LLEG_LY*sin(pi - r['rKnee'])/leg['rightEta']) + r['rHipTilt']
        return leg

    def forward_kinematics(self, leg, r):
        #previously calculated rotation matrix
        z_value = 0.0
        flag = 'Left'
        for i in range(2):
            if r is not None and flag == 'Left':
                theta_y = r['lHipYaw']
                theta_x = r['lHipRoll']
                theta_pl = leg['leftLegAngle']
                lamda = 1
            elif r is not None and flag == 'Right':
                theta_y = r['rHipYaw']
                theta_x = r['rHipRoll']
                theta_pl = leg['rightLegAngle']
                lamda = -1

            a = cos(r['tilt'])*(cos(theta_y)*cos(theta_pl) - sin(theta_x)*sin(theta_y)*sin(theta_pl)) \
                - sin(r['tilt'])*(cos(r['roll'])*(cos(theta_y)*sin(theta_pl) + cos(theta_pl)*sin(theta_x)*sin(theta_y)) + sin(r['roll'])*cos(theta_x)*sin(theta_y))
            b = sin(r['roll'])*(cos(theta_y)*sin(theta_pl) + cos(theta_pl)*sin(theta_x)*sin(theta_y)) - cos(r['roll'])*cos(theta_x)*sin(theta_y)
            c = sin(r['tilt'])*(cos(theta_y)*cos(theta_pl) - sin(theta_x)*sin(theta_y)*sin(theta_pl)) \
                + cos(r['tilt'])*(cos(r['roll'])*(cos(theta_y)*sin(theta_pl) + cos(theta_pl)*sin(theta_x)*sin(theta_y)) + sin(r['roll'])*cos(theta_x)*sin(theta_y))
            d = eta*(cos(theta_y)*sin(theta_pl) + cos(theta_pl)*sin(theta_x)*sin(theta_y))
            e = cos(r['tilt'])*(cos(theta_pl)*sin(theta_y) + cos(theta_y)*sin(theta_x)*sin(theta_pl)) \
                - sin(r['tilt'])*(cos(r['roll'])*(sin(theta_y)*sin(theta_pl) - cos(theta_y)*cos(theta_pl)*sin(theta_x)) - sin(r['roll'])*cos(theta_x)*cos(theta_y))
            f = sin(r['roll'])*(sin(theta_y)*sin(theta_pl) - cos(theta_y)*cos(theta_pl)*sin(theta_x)) + cos(r['roll'])*cos(theta_x)*cos(theta_y)
            g = sin(r['tilt'])*(cos(theta_pl)*sin(theta_y) + cos(theta_y)*sin(theta_x)*sin(theta_pl)) \
                + cos(r['tilt'])*(cos(r['roll'])*(sin(theta_y)*sin(theta_pl) - cos(theta_y)*cos(theta_pl)*sin(theta_x)) - sin(r['roll'])*cos(theta_x)*cos(theta_y))
            h = lamda*ty + eta*(sin(theta_y)*sin(theta_pl) - cos(theta_y)*cos(theta_pl)*sin(theta_x))
            i = sin(r['tilt'])*(sin(r['roll'])*sin(theta_x) - cos(r['roll'])*cos(theta_x)*cos(theta_pl)) \
                - cos(r['tilt'])*cos(theta_x)*sin(theta_pl)
            j = cos(r['roll'])*sin(theta_x) + sin(r['roll'])*cos(theta_x)*cos(theta_pl)
            k = - cos(r['tilt'])*(sin(r['roll'])*sin(theta_x) - cos(r['roll'])*cos(theta_x)*cos(theta_pl)) \
                - sin(r['tilt'])*cos(theta_x)*sin(theta_pl)
            l = z_value - TRUNK_LY/2 + eta*cos(theta_x)*cos(theta_pl)

            R = array([[a, b, c, d,], \
                       [e, f, g, h,], \
                       [i, j, k, l,], \
                       [0, 0, 0, 1]])

            V = array([[0, 0, 0, 1]])
            if flag == 'Left':
                leftLegPos = dot(R,V)
                print leftLegPos
                flag = 'Right'
            else:
                rightLegPos = dot(R,V)
                print rightLegPos
            return

    def foot_position(self, r):
        leg = legLength(r)
        forward_kinematics(leg, r)
        return



#       def forward_kinematics(self,rightJointAngles,leftJointAngles):
#        #measures which foot is closer to the ground
#        rightFootHeight = (ULEG_LY*cos(rightJointAngles['hipPitch']) + \
#                           LLEG_LY*cos(rightJointAngles['knee'])) * \
#                           cos(rightJointAngles['hipRoll'])
#        leftFootHeight = (ULEG_LY*cos(leftJointAngles['hipPitch']) + \
#                          LLEG_LY*cos(leftJointAngles['knee'])) * \
#                          cos(leftJointAngles['hipRoll'])
#        #calculates pendulum states
#        if rightFootHeight > leftFootHeight:
#            lamda = -1
#            cx = LLEG_LY*cos(rightJointAngles['anklePitch']) + \
#                 ULEG_LY*cos(rightJointAngles['knee'])
#            cxDot = (cx - self.cx)/self.T
#            self.cx = cx
#            cy = (ULEG_LY*cos(rightJointAngles['hipPitch']) + \
#                  LLEG_LY*cos(rightJointAngles['knee'])) * \
#                  lamda*cos(rightJointAngles['ankleRoll']) + lamda*TRUNK_LX/2
#            cyDot = (cy - self.cy)/self.T
#            self.cy = cy
#        else:
#            lamda = 1
#            cx = LLEG_LY*cos(rightJointAngles['anklePitch']) + \
#                 ULEG_LY*cos(rightJointAngles['knee'])
#            cxDot = (cx - self.cx)/self.T
#            self.cx = cx
#            cy = (ULEG_LY*cos(rightJointAngles['hipPitch']) + \
#                  LLEG_LY*cos(rightJointAngles['knee'])) * \
#                  lamda*cos(rightJointAngles['ankleRoll']) + lamda*TRUNK_LX/2
#            cyDot = (cy - self.cy)/self.T
#        #still need to rotate accordingly to IMU angle and go through a filter
#        return cx,cxDot,cy,cyDot,lamda
        
