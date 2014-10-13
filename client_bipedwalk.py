import sys
import time
from math import *
from controller import UDPCtrlClient
from omniwalk import *
import pygame
from pygame.locals import *

# checks for command line arguments
if len(sys.argv) != 4:
    print 'Usage:', sys.argv[0],'server_url server_port client_port'
    sys.exit(0)

# creates the UDP client interface
c = UDPCtrlClient(sys.argv[1], sys.argv[2], sys.argv[3])

# Init PyGame
pygame.init()
pygame.display.set_mode((150,150))
screen = pygame.display.get_surface()

#Init walking modules
control = ControlInterface()
motion = MotionPattern()
leg = LegInterface()

VX = 0.0
VY = 0.0
VPHI = 0.0
quit = False

while not quit:
    for e in pygame.event.get():
        if e.type == QUIT:
            quit = True
        elif e.type == KEYDOWN:
            if e.key == K_w:
                print "increase forward speed"
                VX = VX + 0.025
            if e.key == K_s:
                print "decrease forward speed"
                VX = VX - 0.025
            if e.key == K_a:
                print "increase lateral speed"
                VY = VY + 0.025
            if e.key == K_d:
                print "decrease lateral speed"
                VY = VY - 0.025
            if e.key == K_q:
                print "decrease rotational speed"
                VPHI = VPHI + 0.025
            if e.key == K_e:
                print "decrease rotational speed"
                VPHI = VPHI - 0.025

    Vx,Vy,Vphi,tau,start = control.full_control(VX,VY,VPHI)
    # calculates angles for the left leg
    angles = motion.final_motion(Vx,-Vy,-Vphi, tau, 1, start)
    rightJointAngles = leg.joint_angles(angles,1)
    # calculates angles for the right leg
    angles = motion.final_motion(Vx,Vy,Vphi,(tau+2*pi) % (2*pi) - pi, -1, start)
    leftJointAngles = leg.joint_angles(angles,-1)

    # prepare the omniwalk commands to be sent
    # to the server
    command = {'lHipRoll':leftJointAngles['hipRoll'],\
               'rHipRoll':rightJointAngles['hipRoll'],\
               'lHipTilt':-leftJointAngles['hipPitch'],\
               'rHipTilt':-rightJointAngles['hipPitch'],\
               'lHipYaw':leftJointAngles['hipYaw'],\
               'rHipYaw':rightJointAngles['hipYaw'],\
               'lKnee':-leftJointAngles['knee'],\
               'rKnee':-rightJointAngles['knee'],\
               'lAnkleRoll':leftJointAngles['ankleRoll'],\
               'rAnkleRoll':rightJointAngles['ankleRoll'],\
               'lAnkleTilt':-leftJointAngles['anklePitch'],\
               'rAnkleTilt':-rightJointAngles['anklePitch'],\
               }
    print command
    # sends the desired target angles to server, and
    # receive back in r all the current sensor readings
    r = c.sendCommand(command)
    # prints the tilt sensor reading
    if r is not None and 'tilt' in r:
        print 'Tilt sensor reading: %+.3f' % r['tilt']

    time.sleep(0.005)
    pygame.display.update()

