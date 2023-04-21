"""lab2_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
import math

# create the Robot instance.
robot = Robot()

# Numerical Constants
pi =  3.1416
wheelRadius = 0.8

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

robot.step(timestep)
initLPos = leftposition_sensor.getValue()
initRPos = rightposition_sensor.getValue()
print("Left Position Sensor (init): " + str(initLPos))
print("Right Position Sensor (init): " + str(initRPos))

# Robot Functions
def getCounts():
    leftCount = leftposition_sensor.getValue()
    rightCount = rightposition_sensor.getValue()
    return (leftCount, rightCount)
    
def resetCounts():
    global initLPos
    global initRPos
    initLPos = leftp
    initRPos = rightp

def setSpeedsIPS(ipsLeft, ipsRight):
    leftSpeed = ipsLeft / wheelRadius
    rightSpeed = ipsRight / wheelRadius
    if (abs(leftSpeed) > 6.28 or abs(rightSpeed) > 6.28):
        print('Target speed exceeds maximum speed!')
    else:
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    return

def setSpeedsVW(V, W):
    if (W != 0):
        turnRadius = V / W
        ipsLeft = W * (turnRadius - wheelSeparation / 2)
        ipsRight = W * (turnRadius + wheelSeparation / 2)
        setSpeedsIPS(ipsLeft, ipsRight)
    else:
       setSpeedsIPS(V, V) 
    return

def moveXV(X, V):  
    if((leftp - initLPos) < (X / wheelRadius)):
        setSpeedsVW(V, 0)
    else:
        setSpeedsVW(0, 0)
    return
    
def rightTurn(goal):
    curr = imu.getRollPitchYaw()[2]
    # if(goal == -pi):
        # cond = curr < -goal and  : imu.getRollPitchYaw()[2] < goal
    if(abs(curr) < (abs(goal) + 0.02) and abs(curr) > (abs(goal) - 0.02)):
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
    elif(abs(curr) < (abs(goal) + 0.1) and abs(curr) > (abs(goal) - 0.1)):
        leftMotor.setVelocity(1)
        rightMotor.setVelocity(-1)
    else:    
        leftMotor.setVelocity(2.5)
        rightMotor.setVelocity(-2.5)
    return

def rectangle(H, W, X, Y, state):
    if(state == 0):        # Traversing north
        moveXV(H/2, X)
    elif(state == 1):      # First turn
        resetCounts()
        print(initLPos)
        rightTurn(-pi/2)
    elif(state == 2):      # Traversing East
        moveXV(W, X)
    elif(state == 3):      # Second Turn
        rightTurn(pi)
        resetCounts()
    elif(state == 4):      # Traversing South
        moveXV(H, X)
    elif(state == 5):      # Third Turn
        rightTurn(pi/2)
        resetCounts()
    elif(state == 6):      # Traversing West
        moveXV(W, X)
    elif(state == 7):      # Fourth Turn
        rightTurn(0)
        resetCounts()
    elif(state == 8):      # Traversing North
        moveXV(H/2, X)
    else:
        pass
    return

def computeX(H, W, Y):
    return (2*H + 2*W) / (Y - 4)
    
    
# Required variables

state = 0
dToGoal = 0
H, W, Y = 10, 10, 3
X = computeX(H, W, Y)
print(X)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    #leftMotor.setVelocity(6)
    #rightMotor.setVelocity(6.28)
    
    leftp = leftposition_sensor.getValue()
    rightp = rightposition_sensor.getValue()
    
    print("Left position sensor: " + str(leftp))
    print("Right position sensor: " + str(rightp))
    print("imu: "+ str(imu.getRollPitchYaw()[2]))
    print("state: " + str(state))
    print("Velocity: " + str(leftMotor.getVelocity()) + "\n\n")

    # Enter here functions to send actuator commands
    rectangle(H, W, X, Y, state)
    if(leftMotor.getVelocity() == 0.0 and rightMotor.getVelocity() == 0.0):
        state += 1
    pass

# Enter here exit cleanup code.
