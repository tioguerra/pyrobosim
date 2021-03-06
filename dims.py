# Configuration parameters for the
# simulation

# Density in kg/m3
DENSITY = 500 # wood

# Humanoid
# All lengths in meters
LEG_GAP = 0.180
FOOT_LX = 0.150
FOOT_LZ = 0.250
FOOT_LY = 0.025
FOOT_JPOS = (0.0, 0.0, FOOT_LZ/4.0)
LLEG_LX = 0.080
LLEG_LZ = 0.040
LLEG_LY = 0.280
LLEG_JPOS = (0.0, 0.0, 0.0)
ULEG_LX = 0.080
ULEG_LZ = 0.040
ULEG_LY = 0.280
ULEG_JPOS = (0.0, 0.0, 0.0)
TRUNK_LX = 0.280
TRUNK_LZ = 0.100
TRUNK_LY = 0.350
TRUNK_JPOS = (0.0, 0.0, 0.0)

# Quadruped
# All lengths in meters
QTRUNK_LX = 0.280
QTRUNK_LZ = 0.420
QTRUNK_LY = 0.150
QFOOT_RADIUS = 0.050
QFOOT_LY = 0.020
QLLEG_LX = 0.080
QLLEG_LZ = 0.080
QLLEG_LY = 0.250
QULEG_LX = 0.250
QULEG_LZ = 0.080
QULEG_LY = 0.080

# Gravity
GRAVITY = (0, -9.81, 0)
ERP = 0.8 # make this as large as possible
CFM = 0.00001 # make this as small as possible
BOUNCE = 0.1
MU = 55.0

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

FLOOR_IMAGE = 'images/floor.jpg'
ROBOT_IMAGE = 'images/rough.jpg'

# Torque in N.m
MAX_KNEE_TORQUE = 150.0
MAX_ANKLE_TORQUE = 150.0
MAX_HIP_TORQUE = 50.0

QMAX_KNEE_TORQUE = 50.0
QMAX_ANKLE_TORQUE = 50.0
QMAX_HIP_TORQUE = 50.0

# PID gains

HIPROLL_PID = 50.0, 10.0, 4.50  #25.0, 15.0, 2.50
HIPTILT_PID = 50.0, 15.0, 3.50  #25.0, 15.0, 2.50
HIPYAW_PID = 50.0, 10.0, 2.50    #15.0, 2.50, 0.50
KNEE_PID = 150.0, 25.0, 2.00    #150, 10.0, 2.50 - 150.0, 25.0, 3.50
ANKLE_PID = 150.0, 35.0, 1.50    #150.0, 10.0, 1.50

QHIPROLL_PID = 50.0, 10.0, 2.50
QHIPYAW_PID = 50.0, 10.0, 2.50
QKNEE_PID = 50.0, 10.0, 2.50

EPS = 2e-52 # http://www.mathworks.com/help/techdoc/ref/eps.html

FRAMERATE = 50.0
TIME_STEP = 0.001

