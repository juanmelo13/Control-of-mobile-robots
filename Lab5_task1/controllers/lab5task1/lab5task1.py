# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Measurements and constants
pi = math.pi
wheelRadius = 1.6 / 2.0
wheelSeparation = 2.28

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Variables
Kp = 2
Cmax = 5
Cmin = -5
r = -5

grid = [['.'] * 4 for _ in range(4)]

orient = 0
square = -1
x, y = -100, -100
vel = 0

state = 0
direction = 0
nextCol = -5

debug = False
# Functions
def facing(ori, target):
    return ori < target + 0.01 and ori > target - 0.01

def inside(coord, target):
    return coord < target + 0.015 and coord > target - 0.015

def printGrid():
    global grid
    for r in grid:
        print(' '.join(r))

def printPose():
    global x, y, square, orient
    print('Pose: %4.1f, %4.1f  %2d  %4.1f' % (x, y, square, orient))

def getOrientation(imu):
    ori = imu.getRollPitchYaw()[2] + pi
    return ori

def getSquare(x, y):
    if x < -20:
        return -1
    else:
        return (((x + 20) // 10) + 1 + ((y - 20) // -10) * 4)

def getGrid(sq):
    i = sq % 4 - 1
    j = (sq - 1) // 4
    return int(j), int(i)

def getCoords():
    global x, y
    y = 20 - getInches(frontDistanceSensor) - 1.09
    x = 20 - getInches(rightDistanceSensor) - 1.09

def getInches(sensor):
    return sensor.getValue() * 39.37
    
def setSpeedsIPS(ips):
    global vel
    vel = ips
    speed = ips / wheelRadius
    if (abs(speed) > 6.28):
        print('Target speed exceeds maximum speed!')
    else:
        leftMotor.setVelocity(speed)
        rightMotor.setVelocity(speed)
    return

def setAngularSpeed(o):
    leftMotor.setVelocity((o * wheelSeparation / 2) / wheelRadius)
    rightMotor.setVelocity(-(o * wheelSeparation / 2) / wheelRadius)

def E(x, R):
    return R - x

def U(E):
    return Kp * E

def fSat(vel):
    if vel > Cmax:
        return Cmax
    elif vel < Cmin:
        return Cmin
    else: 
        return vel

def rotfSat(vel):
    if vel > 2:
        return 2
    elif vel < -2:
        return 2
    else: 
        return vel

def Ur(x, R):
    return fSat(U(E(x, R))) 

def rotUr(x, R):
    return rotfSat(U(E(x, R)))

def faceDirection(ori, dir):
    setAngularSpeed(-rotUr(ori, dir))

def trilateration(orientation):
    setAngularSpeed(2)
    if camera.getRecognitionObjects():
        target = camera.getRecognitionObjects()[0]
        if target.get_colors() == [1.0, 1.0, 0.0] and target.get_position()[1] > -0.1 and target.get_position()[1] < 0.1:
            r1 = getInches(frontDistanceSensor) + 1.09
            x = -20 + math.cos(2 * pi - orientation) * r1
            y = 20 - math.sin(2 * pi - orientation) * r1
    return x, y


def navigation(ori, x, y, square):
    global state, direction, nextCol
    # Calibration
    if state == -1:
        faceDirection(ori, 4.71)
        if facing(ori, 4.71):
            state = -2
            setAngularSpeed(0)
            return
    elif state == -2:
        getCoords()
        square = getSquare(x, y)
        if not inside(y, -15):
            state = -3
            return
        elif not inside(x, -15):
            state = 4
            return
    elif state == -3:
        if inside(y , -15):
            setSpeedsIPS(0)
            state = -4
            return
        setSpeedsIPS(Ur(y, -15))
    elif state == -4:
        if not facing(ori, 3.14):
            faceDirection(ori, 3.14)
        else:
            if inside(x, -15):
                setSpeedsIPS(0)
                state = 1
                return
            setSpeedsIPS(Ur(x, -15))
    # Initial State
    elif state == 0:
        if square == -1:
            state = -1
        elif square == 13:
            state = 1
    # Navigation from block 13
    elif state == 1:
        if facing(ori, 4.71):
            state = 2
            return
        faceDirection(ori, 4.71)
    elif state == 2:
        if direction == 0:
            if inside(y, 15):
                state = 3
                setSpeedsIPS(0)
                direction = 1
                return
            setSpeedsIPS(Ur(y, 15))
        else:
            if inside(y, -15):
                if square == 16:
                    state = 5
                    setSpeedsIPS(0)
                    return
                state = 3
                setSpeedsIPS(0)
                direction = 0
                return
            setSpeedsIPS(Ur(y, -15))
    elif state == 3:
        if facing(ori, 3.14):
            state = 4
            return
        faceDirection(ori, 3.14)
    elif state == 4:
        if inside(x, nextCol):
            state = 1
            setSpeedsIPS(0)
            nextCol += 10
            return
        setSpeedsIPS(Ur(x, nextCol))
    elif state == 5:
        pass
    return


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    orient = getOrientation(imu)
    if getSquare(x, y) != square:
        square = getSquare(x, y)
        indexes = getGrid(square)
        grid[indexes[0]][indexes[1]] = 'X'
        printGrid()
        printPose()

    if debug:
        print("IMU:", getOrientation(imu))
        print("Square:", square)
        print("Velocity:", vel)
        print("State", state)
        printPose()
    
    # Process sensor data here.

    x -= math.cos(getOrientation(imu)) * vel * timestep / 1000
    y -= math.sin(getOrientation(imu)) * vel * timestep / 1000

    # Enter here functions to send actuator commands, like:
    navigation(orient, x, y, square)
    pass

# Enter here exit cleanup code.
