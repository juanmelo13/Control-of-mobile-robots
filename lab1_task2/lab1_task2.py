"""lab1_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Measurements and constants
t = 0.0    # Time elapsed (s)
pi = 3.1416
wheelRadius = 1.6 / 2.0
wheelSeparation = 2.28

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

robot.step(timestep)

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')

# Define functions

def initEncoders():    
    leftposition_sensor.enable(timestep)
    rightposition_sensor.enable(timestep)
    return

def resetCounts():
    return
    
def getCounts():
    leftCount = leftposition_sensor.getValue()
    rightCount = leftposition_sensor.getValue()
    return (leftCount, rightCount)
    
def getSpeeds():
    if (t == 0):
        return (0, 0)
    counts = getCounts()
    speed = tuple((count / (2 * pi)) / t for count in counts)
    return speed

def setSpeedsRPS(rpsLeft, rpsRight):
    if (abs(rpsLeft) > 1 or abs(rpsRight) > 1):
        print('Target speed exceeds maximum speed!')
    else:        
        leftMotor.setVelocity(rpsLeft * 2 * pi)
        rightMotor.setVelocity(rpsRight *2 * pi)
    return

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
        r = V / W
        ipsLeft = W * (r - (wheelSeparation / 2))
        ipsRight = W * (r + (wheelSeparation / 2))
        setSpeedsIPS(ipsLeft, ipsRight)
    else:
       setSpeedsIPS(V, V) 
    return

    
def turnRV(R1, V):
    if (R1 == 0):
        print("Invalid radius of 0")
    else:
        setSpeedsVW(V, V/R1)
    return

# Main loop:
# - perform simulation steps until Webots is stopping the controller -

r1 = -10   # Target Radius (in)
v = 4    # Target Velocity (in/s)

turnRV(r1, v)
initEncoders()

print("Stopping Time: " + str(abs(r1 * 2 * pi)/abs(v)))

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    
    # print("Left position sensor: " +str(leftposition_sensor.getValue()))
    # print("Right position sensor: " +str(rightposition_sensor.getValue()))
    print("Time elapsed: " + str(t))
    print("Distance Traveled: " + str(abs(v) * t))
    print(str(getSpeeds()))
    
    # Process sensor data here.
    
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

    t += timestep / 1000.0

# Enter here exit cleanup code.
