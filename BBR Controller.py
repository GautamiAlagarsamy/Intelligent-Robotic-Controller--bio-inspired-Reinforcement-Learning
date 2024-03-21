from controller import Robot, DistanceSensor, Motor , LightSensor
import numpy as np
###
###
### Line-follower part cite by https://github.com/felipenmartins/Robotics-Simulation-Labs/blob/70f8e0b6e3321e83a9a61aa48c4984d1371d62af/Lab2/
### Odometry-based Localization part cite by https://github.com/felipenmartins/Robotics-Simulation-Labs/blob/70f8e0b6e3321e83a9a61aa48c4984d1371d62af/Lab3/
### Avoid obstacles part has no cite
###
###
TIME_STEP = 64
MAX_SPEED = 5

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = robot.getBasicTimeStep()/1000.0
# states
states = ['forward', 'turn_right', 'turn_left','Mid','stop1','stop2']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 5
x=-0.685
y=-0.65
phi = 1.58

dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0565    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]
#-------------------------------------------------------
# Initialize devices
ls = []
lsNames = ['ls0','ls1','ls2','ls3','ls4','ls5','ls6','ls7']
for i in range(8):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(timestep)

# distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)


encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
oldEncoderValues = []

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
light=0

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    #Encoder values indicate the angular position of the wheel in radians
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t
    return wl, wr

def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)
    return u, w

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    phi_avg = (phi_old + phi)/2   
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    delta_x = u * np.cos(phi_avg) * delta_t
    delta_y = u * np.sin(phi_avg) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Update sensor readings
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())
    
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())
        
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())
    
    light_left = lsValues[5] 
    # Process sensor data
    line_right = gsValues[0] > 600
    line_center =gsValues[1] > 600
    line_left = gsValues[2] > 600
   
    distance_middle_right = psValues[0] >228
    distance_right = psValues[1] > 2000
    distance__left = psValues[6] > 2000
    distance_middle_left = psValues[7] > 195
    
    # Implement the line-following state machine
    if current_state == 'forward':
        # Action for the current state: update speed variables
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED

        if distance_middle_right or  distance_middle_left or distance__left or distance_right:  
            current_state = 'avoid'
            counter = 0
        if light >= 1 and 0.125 <= x <= 0.135 and -0.075 <= y <= -0.065:
            current_state = 'stop1'
            counter = 0
        if light < 1 and 0.235 <= x <= 0.245 and -0.005 <= y <= 0.005:
            current_state = 'stop2'
            counter = 0
        # check if it is necessary to update current_state
        if line_right:
            current_state = 'turn_right'
            counter = 0
        if  line_center:
            current_state = 'Mid'
            counter = 0       
        elif line_left:
            current_state = 'turn_left'
            counter = 0
           
    if current_state == 'turn_right':
        # Action for the current state: update speed variables
        leftSpeed = 1 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'
                   
    if  light_left == 0:      
        light +=1

    if current_state == 'stop1':
        if counter <1:
            leftSpeed = - MAX_SPEED
            rightSpeed = MAX_SPEED
            light=0
        if 1<counter <22:
            leftSpeed =  MAX_SPEED
            rightSpeed = MAX_SPEED
            light=0            
        if 22<counter<552:
            leftSpeed =0
            rightSpeed=0
        if 552<counter:
            counter = 0
            x=-0.685
            y=-0.65
            phi = 1.58
            current_state = 'forward'
    if current_state == 'stop2':
        if counter <1:
            leftSpeed =  MAX_SPEED
            rightSpeed =- MAX_SPEED
            light=0
        if 1<counter <22:
            leftSpeed =  MAX_SPEED
            rightSpeed = MAX_SPEED
            light=0 
        if 22<counter<500:  
            leftSpeed =0
            rightSpeed=0
        if 500<counter:
            counter = 0
            x=-0.685
            y=-0.65
            phi = 1.58
            current_state = 'forward'                 
         
                          
    if current_state == 'Mid':
        if light >= 1:
            if counter<7:   
                leftSpeed = -1.0* MAX_SPEED
                rightSpeed = 1.0 * MAX_SPEED
            if 7< counter:
                counter = 0
                current_state = 'forward'
        else:
            if counter<7:   
                leftSpeed = 1.0* MAX_SPEED
                rightSpeed = -1.0 * MAX_SPEED
            if 7< counter:
                counter = 0
                current_state = 'forward'

    if current_state == 'turn_left':
        # Action for the current state: update speed variables
        leftSpeed = 0.4 * MAX_SPEED
        rightSpeed = 1 * MAX_SPEED
        if distance_middle_right or  distance_middle_left :  
            current_state = 'avoid'
            counter = 0
        if counter == COUNTER_MAX:
            current_state = 'forward'
             
    if current_state == 'avoid': 
        if light >= 1:
            if counter<7:
             leftSpeed =-MAX_SPEED
             rightSpeed =-MAX_SPEED
         
            if 7<counter<40:
             leftSpeed =-MAX_SPEED
             rightSpeed =MAX_SPEED
            if 40<counter<85:
             leftSpeed =MAX_SPEED
             rightSpeed =MAX_SPEED
            if 85<counter<115:
             leftSpeed =MAX_SPEED
             rightSpeed =-MAX_SPEED
            if 115<counter<175:
             leftSpeed = MAX_SPEED
             rightSpeed =MAX_SPEED
            if 175<counter<212:
             leftSpeed = MAX_SPEED
             rightSpeed =-MAX_SPEED
            if 212<counter<280:
             leftSpeed = MAX_SPEED
             rightSpeed =MAX_SPEED 
     
            if 280<counter:
             counter = 0
             current_state = 'forward'
            
        else:
            if counter<7:
             leftSpeed =-MAX_SPEED
             rightSpeed =-MAX_SPEED
            if 7<counter<42:
             leftSpeed =MAX_SPEED
             rightSpeed =-MAX_SPEED
            if 42<counter<85:
             leftSpeed =MAX_SPEED
             rightSpeed =MAX_SPEED
            if 85<counter<115:
             leftSpeed = -MAX_SPEED
             rightSpeed =  MAX_SPEED
            if 115<counter<285:
             leftSpeed = MAX_SPEED
             rightSpeed =  MAX_SPEED
            if 185<counter<220:
             leftSpeed = -MAX_SPEED
             rightSpeed =  MAX_SPEED
            if 220<counter<275:
             leftSpeed = MAX_SPEED
             rightSpeed =  MAX_SPEED

            if 275<counter:
             counter = 0
             current_state = 'forward'             
        
    # increment counter
    counter += 1
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)
    
    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
    oldEncoderValues = encoderValues
    
    #print('Counter: '+ str(counter), gsValues[0], gsValues[1], gsValues[2])
    print('Counter: '+ str(counter) + '. Current state: ' + current_state)
    print(f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')

    # Set motor speeds with the values defined by the state-machine
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
