"""lab2_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# Numerical Constants
pi =  3.1416
wheelRadius = 0.8
wheelSeparation = 2.28

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

# Robot functions
def getIMUDegrees():
    return imu.getRollPitchYaw()[2] * 360 / (2*pi)

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

def turnRV(r, V):
    if (r == 0):
        print("Invalid radius of 0")
    else:
        setSpeedsVW(V, V/r)
    return

def circles(R1, R2, X):
    global state
    if(state == 0):        # Begining traverse first circle
        turnRV(-R1, X)
        if(imuRead < -80 and imuRead > -86):
            state = 1
    elif(state == 1):      # Wait till getting to 0
        if(imuRead < 0.8 and imuRead > -0.8):
            state = 2
    elif(state == 2):
        turnRV(R2, X)
        if(imuRead < 81 and imuRead > 79):
            state = 3
    else:
        if(imuRead < 1 and imuRead > -1):
            setSpeedsIPS(0, 0)
        pass
    return
    
def computeX(R1, R2, Y):
    return (R1 + R2) * 2 * pi / Y
    
    
# Required variables
state = 0
R1, R2, Y = 5, 10, 30
X = computeX(R1, R2, Y)
X = 3
print(X)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    leftp = leftposition_sensor.getValue()
    rightp = rightposition_sensor.getValue()
    imuRead = getIMUDegrees()
    
    print("Left position sensor: " + str(leftp))
    print("Right position sensor: " + str(rightp))
    print("imu: "+ str(imu.getRollPitchYaw()[2]) + " (" + str(imuRead) + " deg)")
    print("state: " + str(state))
    print("Velocity: " + str(leftMotor.getVelocity()) + "\n\n")

    circles(R1, R2, X)
    pass

# Enter here exit cleanup code.
