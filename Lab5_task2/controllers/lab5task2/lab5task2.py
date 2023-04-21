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
walls = ['WWOW', 'OWOW', 'OWOO', 'OWWO', 'WWOO', 'OWWO', 'WOWO', 'WOWO', 'WOOO', 'OOWW', 'WOWW', 'WOWO', 'WOOW', 'OWOW', 'OWOW', 'OOWW']
path = [13, 9, 10, 6, 5, 13, 14, 15, 16, 12, 8, 4, 3, 17, 11, 3, 2, 1]

orient = 0
square = -1
squareWalls = list('OOOO')
pastWalls = []
dir = 0

x, y = -150, -150
vel = 0

state = 0
prevState = 0
nextCord = 0

debug = False
# Functions
def findCell(wall, dir):
    global walls
    matches = []
    if len(wall) == 1:
        for w in walls:
            if ''.join(wall[0]) == w:
                matches.append(walls.index(w) + 1)
    else:
        for i in range(len(walls)):
            if ''.join(wall[0]) == walls[i] and ''.join(wall[1]) == walls[i + dir]:
                matches.append(i + dir + 1)
    return matches

def setPose(square):
    global x, y
    x = -15 + ((square - 1) % 4) * 10
    y = 15 - ((square - 1) // 4) * 10

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

def visited(square):
    global grid
    i = getGrid(square)
    return grid[i[0]][i[1]] == 'X'
    
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

def stop():
    setSpeedsIPS(0)
    setAngularSpeed(0)

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

def navigation(ori, x, y, square):
    global state, dir, nextCord, squareWalls, pastWalls, prevState

    # Calibration
    if state == -1:
        faceDirection(ori, 4.71)
        if facing(ori, 4.71):
            state = -2
            setAngularSpeed(0)
            return
    elif state == -1.5:
        if len(findCell(pastWalls, dir)) == 1:
            state = 1
            setPose(findCell(pastWalls, dir)[0])
            return
        elif len(findCell(pastWalls, dir)) > 0:
            state = -5
            return
    elif state == -2:
        squareWalls = list('OOOO')
        if getInches(frontDistanceSensor) < 5:
            squareWalls[1] = 'W'
        if getInches(rightDistanceSensor) < 5:
            squareWalls[2] = 'W'
        if getInches(leftDistanceSensor) < 5:
            squareWalls[0] = 'W'
        state = -3
    elif state == -3:
        faceDirection(ori, 1.57)
        if facing(ori, 1.57):
            state = -4
            setAngularSpeed(0)
            return
    elif state == -4:
        if getInches(frontDistanceSensor) < 5:
            squareWalls[3] = 'W'
        pastWalls.append(squareWalls)
        state = -1.5
    elif state == -5:
        prevState = -1
        if squareWalls[0] == 'O':
            state = 2 # Move west
            dir = -1
            nextCord = x - 10
            return
        elif squareWalls[2] == 'O':
            state = 3 # Move east
            dir = 1
            nextCord = x + 10 * (next - square)
            return
        elif squareWalls[1] == 'O':
            state = 4 # Move north
            dir = -4
            nextCord = y + 10
            return 
        elif squareWalls[3] == 'O':
            state = 5 # Move south
            dir = 4
            nextCord = y - 10
            return
    elif state == 0:
        if square == -1:
            state = -1
        else:
            state = 1
    elif state == 1:
        prevState = 1
        next = path[(path.index(square) + 1) % 18]
        if visited(next):
            state = 6
            return
        if square - next == 1:
            state = 2 # Move west
            nextCord = x - 10
            return
        elif square - next == -1 or next - square == 3:
            state = 3 # Move east
            nextCord = x + 10 * (next - square)
            return
        elif square - next == 4:
            state = 4 # Move north
            nextCord = y + 10
            return 
        elif square - next == -4:
            state = 5 # Move south
            nextCord = y - 10
            return
    elif state == 2:
        if not facing(ori, 3.14):
            setSpeedsIPS(0)
            faceDirection(ori, 3.14)
        else:
            if inside(x, nextCord):
                stop()
                state = prevState
                return
            setSpeedsIPS(Ur(x, nextCord))
    elif state == 3:
        if not facing(ori, 3.14):
            setSpeedsIPS(0)
            faceDirection(ori, 3.14)
        else:
            if inside(x, nextCord):
                stop()
                state = prevState
                return
            setSpeedsIPS(Ur(x, nextCord))
    elif state == 4:
        if not facing(ori, 4.71):
            setSpeedsIPS(0)
            faceDirection(ori, 4.71)
        else:
            if inside(y, nextCord):
                stop()
                state = prevState
                return
            setSpeedsIPS(Ur(y, nextCord))
    elif state == 5:
        if not facing(ori, 4.71):
            setSpeedsIPS(0)
            faceDirection(ori, 4.71)
        else:
            if inside(y, nextCord):
                stop()
                state = prevState
                return
            setSpeedsIPS(Ur(y, nextCord))
    elif state == 6:
        stop()
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
        print("PastWalls:", pastWalls)
        printPose()
    
    # Process sensor data here.

    x -= math.cos(getOrientation(imu)) * vel * timestep / 1000
    y -= math.sin(getOrientation(imu)) * vel * timestep / 1000

    # Enter here functions to send actuator commands, like:
    navigation(orient, x, y, square)
    pass

# Enter here exit cleanup code.
