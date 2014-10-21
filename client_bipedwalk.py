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
screen = pygame.display.set_mode((250,250))
pygame.display.set_caption('Robot Control')
background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250,250,250))
font = pygame.font.SysFont("arial", 24)
clk = pygame.time.Clock()

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
        #defines keyboard velocity controls
        elif e.type == KEYDOWN:
            if e.key == K_w:
                VX = VX + 0.25
            if e.key == K_s:
                VX = VX - 0.25
            if e.key == K_a:
                VY = VY + 0.25
            if e.key == K_d:
                VY = VY - 0.25
            if e.key == K_q:
                VPHI = VPHI + 0.25
            if e.key == K_e:
                VPHI = VPHI - 0.25

    #renders the velocities to the pygame screen
    VXText = font.render("VX = %1.3f" %VX, True, (10,10,10))
    VYText = font.render("VY = %1.3f" %VY, True, (10,10,10))
    VPHIText = font.render("VPHI = %1.3f" %VPHI, True, (10,10,10))
    screen.fill((255,255,255))
    screen.blit(VXText,(screen.get_width()/2-VXText.get_width()/2,screen.get_width()/6-VXText.get_height()/2))
    screen.blit(VYText,(screen.get_width()/2-VYText.get_width()/2,3*screen.get_width()/6-VYText.get_height()/2))
    screen.blit(VPHIText,(screen.get_width()/2-VPHIText.get_width()/2,5*screen.get_width()/6-VPHIText.get_height()/2))
    
    Vx,Vy,Vphi,tau,start = control.full_control(VX,VY,VPHI)
    # calculates angles for the left leg
    angles = motion.final_motion(Vx,Vy,-Vphi, tau, 1, start)
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
    # sends the desired target angles to server, and
    # receive back in r all the current sensor readings
    r = c.sendCommand(command)
    # prints the tilt sensor reading
    if r is not None and 'tilt' in r:
        print 'Tilt sensor reading: %+.3f' % r['tilt']

    clk.tick(30)
    pygame.display.update()

