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

orient = 0
square = 13
x, y = -15, -15
vel = 0

state = 0
direction = 0
nextCol = -5

debug = False

# Maze 
#[Visited, North, East, South, West]
grid = [[0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 1, 0, 0, 0],[0, 1, 1, 0, 0], #starting grid
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 1, 1], [0, 0, 0, 1, 0], [0, 0, 0, 1, 0],[0, 0, 1, 1, 0]]

def printMaze(maze):
    print("________________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        print("|  "+ str(maze[x][1]) +"\t  " +str(maze[x+1][1])+"\t  " +str(maze[x+2][1])
              +"\t  " +str(maze[x+3][1])+ "    |")
        print("|" +str(maze[x][4]) + " " +v1+" " + str(maze[x][2])+"\t" +str(maze[x+1][4])+ " " +v2+" " + str(maze[x+1][2])
              +"\t" +str(maze[x+2][4])+ " " +v3+" " + str(maze[x+2][2])
              +"\t" +str(maze[x+3][4]) + " " +v4+" " + str(maze[x+3][2]) +"  |")
        print("|  "+str(maze[x][3]) +"\t  " +str(maze[x+1][3])+"\t  " +str(maze[x+2][3])
              +"\t  " +str(maze[x+3][3])+"    |")
        if(i==3):
            print("|_______________________________________|\n")
        else:
            print("|                                       |")

# Functions
def facing(ori, target, flex=False):
    if flex:
        return ori < target + 0.01 and ori > target - 0.01
    return ori < target + 0.005 and ori > target - 0.005

def inside(coord, target, flex=False):
    if flex:
        return coord < target + 4 and coord > target - 4
    return coord < target + 0.2 and coord > target - 0.2

def printPose():
    global x, y, square, orient
    print('Pose: %4.1f, %4.1f  %2d  %4.2f' % (x, y, square, orient))

def getPose(square):
    x = -15 + ((square - 1) % 4) * 10
    y = 15 - ((square - 1) // 4) * 10
    return x, y

def getOrientation(imu):
    ori = imu.getRollPitchYaw()[2] + pi
    return ori

def getSquare(x, y):
    if x < -20:
        return -1
    else:
        return int(((x + 20) // 10) + 1 + ((y - 20) // -10) * 4)

def getGrid(sq):
    i = sq % 4 - 1
    j = (sq - 1) // 4
    return int(j), int(i)

def insideCell(x, y, square):
    xi, yi = getPose(square)
    return inside(x, xi, True) and inside(y, yi, True)

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

def turn270(ori):
    if facing(ori, 3.14, True):
        return 1.57
    elif facing(ori, 1.57, True):
        return 6.28
    elif facing(ori, 6.28, True):
        return 4.71
    elif facing(ori, 4.71, True):
        return 3.14

def turn90(ori):
    if facing(ori, 3.14, True):
        return 4.71
    elif facing(ori, 1.57, True):
        return 3.14
    elif facing(ori, 6.28, True):
        return 1.57
    elif facing(ori, 4.71, True):
        return 6.28

def checkStop(grid):
    for cell in grid:
        if cell[0] == 0:
            return False
    return True

def scanWalls(square, ori):
    global grid
    cell = grid[square - 1]
    front = getInches(frontDistanceSensor) < 11
    right = getInches(rightDistanceSensor) < 5
    left = getInches(leftDistanceSensor) < 5
    if facing(ori, 3.14, True):
        cell[1] = int(front)
        cell[2] = int(right)
        cell[4] = int(left)
    elif facing(ori, 4.71, True):
        cell[4] = int(front)
        cell[1] = int(right)
        cell[3] = int(left)        
    elif facing(ori, 6.28, True):
        cell[3] = int(front)
        cell[4] = int(right)
        cell[2] = int(left)        
    elif facing(ori, 1.57, True):
        cell[2] = int(front)
        cell[3] = int(right)
        cell[1] = int(left)

def navigation(ori, x, y, square):
    global state, direction, nextCol
    if state == -1:
        print("Error: state -1")
        return
    # Initial State
    elif state == 0:
        if square == -1:
            state = -1
        else:
            state = 1
    # Navigation from known block
    elif state == 1:    # Forward
        if getInches(frontDistanceSensor) < 3.5:
            setSpeedsIPS(0)
            direction = turn270(ori)
            # print('Direction:', direction, 'ORI:', ori)
            state = 2
            return
        elif getInches(leftDistanceSensor) > 10:
            state = 3
            return
        setSpeedsIPS(-Ur(getInches(frontDistanceSensor), -3.5))
    elif state == 2:    # Front wall blocking
        if facing(ori, direction):
            setAngularSpeed(0)
            state = 1
            return
        faceDirection(ori, direction)
    elif state == 3:    # Left left wall
        if inside(x, getPose(square)[0]) and inside(y, getPose(square)[1]):
            setSpeedsIPS(0)
            direction = turn90(ori)
            state = 4
            return
    elif state == 4:
        if facing(ori, direction):
            setAngularSpeed(0)
            setSpeedsIPS(5)
            state = 5
            return
        faceDirection(ori, direction)
    elif state == 5:
        if getInches(leftDistanceSensor) < 5:
            state = 1
            return
    return

x, y = getPose(square)

# First Print
grid[square - 1][0] = 1
printMaze(grid)
printPose()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    orient = getOrientation(imu)
    square = getSquare(x, y)
    # Stopping condition
    if checkStop(grid):
        setSpeedsIPS(0)
        print("Map Completed")
        printMaze(grid)
        printPose()
        continue

    if grid[square - 1][0] == 0:
        cellCenter = getPose(square)
        if insideCell(x, y, square):
            scanWalls(square, orient)
            square = getSquare(x, y)
            grid[square - 1][0] = 1
            printMaze(grid)
            printPose()



    if debug:
        print("IMU:", getOrientation(imu))
        print("Square:", square)
        print("Velocity:", vel)
        print("State", state)
        print("Sensors - Front:", getInches(frontDistanceSensor), "Right:", getInches(rightDistanceSensor), "Left:", getInches(leftDistanceSensor))
        print("Direction:", direction)
        printPose()
    
    # Process sensor data here.

    x += math.sin(getOrientation(imu)) * vel * timestep / 1000
    y -= math.cos(getOrientation(imu)) * vel * timestep / 1000

    # Enter here functions to send actuator commands, like:
    navigation(orient, x, y, square)
    pass

# Enter here exit cleanup code.
