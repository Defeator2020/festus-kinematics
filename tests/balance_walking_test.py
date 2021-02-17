import time
import threading
import numpy as np
from math import sqrt, sin, cos, acos, atan
import signal

import os
import sys
import smbus
import busio
import board

from adafruit_servokit import ServoKit
import adafruit_fxos8700
import adafruit_fxas21002c
from imusensor.filters import kalman 
import I2C_LCD_driver

# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
mag_accel = adafruit_fxos8700.FXOS8700(i2c)
gyro = adafruit_fxas21002c.FXAS21002C(i2c)

# Define the servo controller board and its parameters
kit = ServoKit(channels=16)
for i in range(4, 16):
    kit.servo[i].set_pulse_width_range(1000, 2000)
    kit.servo[i].actuation_range = 120


class Body:
    # Define the size of various system elements (mm)
    length = 90  # Halfway between front to rear leg pivots
    width = 40  # Halfway between hip pivots
    hip_offset = 56
    upper_leg = 108
    lower_leg = 133
    radius = sqrt(length**2 + width**2)  # Distance from center of body to shoulder (mm)
    alpha = atan(width/length)  # Existing angle between +x-axis and radius line about center of body (rad)
    
    marg_offset = [0, 0, 0]
    
    # Define some useful positions
    rest_position = [0, 0, 190, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
    lay_position = [-90, 0, 70, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
    walk_position = [0, 0, 190, 0, 0, 0]  # DEBUG
    
    position = rest_position

class Feet:
    walk_lateral = 45 #56
    
    rest_position = [0, 15, 0, 0, -15, 0, 0, 15, 0, 0, -15, 0]  # x, y, z; rr, rl, fr, fl
    lay_position = [0, 15, 0, 0, -15, 0, 0, 15, 0, 0, -15, 0]  # x, y, z; rr, rl, fr, fl
    walk_position = [0, walk_lateral, 0, 0, -walk_lateral, 0, 0, walk_lateral, 0, 0, -walk_lateral, 0]  # DEBUG
    
    position = rest_position

class Servos:
    # Define various servo properties (offset to make 0 straight down, rest positions, and the flip value to compensate for different servo directions)
    offsets = [0, 0, 0, 0, -68, 76, -78, 78, 19, -26, 28, -28, 11, -2, -9, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips (same for all three)
    positions = [90, 90, 90, 90, 76, 45, 72, 48, 33, 87, 37, 83, 89, 40, 36, 73]
    flip = [1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1]

class Stride:
    # Define various gait parameters (mm)
    cg_x_offset = -10  # Forward of center
    cg_y_offset = 0  # Right of center
    length = 40  # Distance from midpoint to either extreme of step
    height = 70  # Distance from ground to highest control point of Bezier curve
    single_margin = 40  # How far from one side the chassis stays during a single step lean
    steer = np.deg2rad(0)  # Target angle to walk at (clockwise from forward=0) (rad)
    
class IMU():
    # Define various IMU parameters
    # Orientation PID stuff
    # count = 0
    balance_P = 0.01
    balance_D = balance_P/4
    balance_I = balance_D/10

    pitch_prev_error = 0
    roll_prev_error = 0
    pitch_sum_error = 0
    roll_sum_error = 0
    target_pitch = -3
    target_roll = 0
    currTime = time.time()

# Instantiate various objects
body = Body()
feet = Feet()
servos = Servos()
stride = Stride()
imu = IMU()
sensorfusion = kalman.Kalman()
mylcd = I2C_LCD_driver.lcd()
"""
mylcd.backlight(1)
mylcd.lcd_display_string("Initializing", 1, 0) # Add dots animation (ellipses) after this while the IMU is initializing
time.sleep(1)
for i in range(4):
    mylcd.lcd_display_string(".", 1, 12 + i)
    time.sleep(1)
mylcd.lcd_clear()
mylcd.lcd_display_string("System", 1, 5)
mylcd.lcd_display_string("Online", 2, 5)
"""
# KINEMATIC CALCULATIONS
def leg_angles():
    target_x = body.position[0]  # Body lean forward
    target_y = body.position[1]  # Body lean right
    target_z = body.position[2]  # Body get taller
    target_pitch = np.deg2rad(body.position[3])  # Pitch back
    target_roll = np.deg2rad(body.position[4])  # Roll left
    target_yaw = np.deg2rad(body.position[5])  # Yaw clockwise
    feet_position = feet.position
    foot_targets = [0]*12
    
    # Pitch calculation --------------------
    pitch_height_shift = body.length*sin(target_pitch)
    longitudinal_pitch = body.length*(1 - cos(target_pitch))
    
    # Roll calculation --------------------
    roll_height_shift = body.width*sin(target_roll)
    lateral_roll = body.width*(1 - cos(target_roll))
    
    # Yaw calculation --------------------
    longitudinal_yaw = body.radius*cos(body.alpha - target_yaw) - body.length
    lateral_yaw = body.radius*sin(body.alpha - target_yaw) - body.width
    
    
    foot_targets[0] = target_x - feet_position[0] - longitudinal_pitch - longitudinal_yaw
    foot_targets[1] = -target_y + feet_position[1] + lateral_roll - lateral_yaw
    foot_targets[2] = target_z - feet_position[2] - pitch_height_shift + roll_height_shift
    foot_targets[3] = target_x - feet_position[3] - longitudinal_pitch - longitudinal_yaw
    foot_targets[4] = target_y - feet_position[4] + lateral_roll + lateral_yaw
    foot_targets[5] = target_z - feet_position[5] - pitch_height_shift - roll_height_shift
    foot_targets[6] = target_x - feet_position[6] + longitudinal_pitch + longitudinal_yaw
    foot_targets[7] = -target_y + feet_position[7] + lateral_roll + lateral_yaw
    foot_targets[8] = target_z - feet_position[8] + pitch_height_shift + roll_height_shift
    foot_targets[9] = target_x - feet_position[9] + longitudinal_pitch + longitudinal_yaw
    foot_targets[10] = target_y - feet_position[10] + lateral_roll - lateral_yaw
    foot_targets[11] = target_z - feet_position[11] + pitch_height_shift - roll_height_shift
    
    for leg in range(4):
        # Y offset calculation --------------------
        adjusted_z = sqrt(foot_targets[2 + 3*leg]**2 + foot_targets[1 + 3*leg]**2 - body.hip_offset**2)
        hip_angle = np.rad2deg(atan(foot_targets[1 + 3*leg]/foot_targets[2 + 3*leg]) + atan(adjusted_z/body.hip_offset)) - 90
        
        # X offset calculation --------------------
        shoulder_offset = np.rad2deg(atan(foot_targets[0 + 3*leg]/adjusted_z))
        leg_length = sqrt(adjusted_z**2 + foot_targets[0 + 3*leg]**2)
        
        # Z offset calculation --------------------
        shoulder_base_angle = np.rad2deg(acos((body.upper_leg**2 + leg_length**2 - body.lower_leg**2)/(2*body.upper_leg*leg_length)))
        wrist_angle = 180 - np.rad2deg(acos((body.lower_leg**2 + body.upper_leg**2 - leg_length**2)/(2*body.lower_leg*body.upper_leg)))
        
        servos.positions[12 + leg] = (hip_angle + ((-1)**(leg+1))*body.position[4])*servos.flip[12 + leg] + servos.offsets[12 + leg] + 60
        servos.positions[8 + leg] = (shoulder_base_angle + shoulder_offset + body.position[3])*servos.flip[8 + leg] + servos.offsets[8 + leg] + 60
        servos.positions[4 + leg] = (wrist_angle)*servos.flip[4 + leg] + servos.offsets[4 + leg] + 60

# Write positions to servos
def write_to_servos():
    print(servos.positions)  # Print servo angles for debugging
    for i in range(16):
        kit.servo[i].angle = servos.positions[i]

# Move to default (rest) position, as defined above
def reset_pose():
    # Set rest position and orientation for chassis and feet
    body.position = body.rest_position
    feet.position = feet.rest_position
    move()

# Calculate the servo angles and write them to the servos, checking for errors
def move():
    try:
        leg_angles()
    except:
        print("Math domain error!")
    
    try:
        write_to_servos()
    except:
        print("Angle out of range!")


# Manage the synchronized movement of all four legs for various gaits --------------------
def walk():
    lean_increments = 10
    step_increments = 20
    slide_increment = stride.length/step_increments

    for i in range(4):
        
        if i == 0:
            feet_set = (1, 2, 3, 0)  # Determines which foot is lifting and which are sliding (lift, slide, slide, slide)
            
            # lean phase
            lean_step = (feet.position[1] + body.width - stride.single_margin - body.position[1] - stride.cg_y_offset)/lean_increments
            for i in range(lean_increments):
                body.position[1] += lean_step
                move()
        
        if i == 1:
            feet_set = (3, 0, 1, 2)
            
        if i == 2:
            feet_set = (0, 1, 2, 3)
        
            # lean phase
            lean_step = (feet.position[4] - body.width + stride.single_margin - body.position[1] - stride.cg_y_offset)/lean_increments
            for i in range(lean_increments):
                body.position[1] += lean_step
                move()
        
        if i == 3:
            feet_set = (2, 3, 0, 1)
        
        # Assign target points for step
        foot_start = feet.position[0 + 3*feet_set[0]]
        foot_end = stride.length
        
        # leg move phase
        for j in range(step_increments):
            t = (j/(step_increments - 1))  # Point along curve, from 0 to 1
        
            # Move the foot that is lifting this cycle
            feet.position[0 + 3*feet_set[0]] = foot_start*(1-t)**3 + (3/2)*(foot_start)*t*(1-t)**2 + 6*foot_end*(1-t)*t**2 + foot_end*t**3
            feet.position[2 + 3*feet_set[0]] = (9/4)*stride.height*t*(1-t)**2 + 3*stride.height*(1-t)*t**2
            
            # Move the feet that are sliding this cycle
            feet.position[0 + 3*feet_set[1]] -= slide_increment
            feet.position[2 + 3*feet_set[1]] = 0
            feet.position[0 + 3*feet_set[2]] -= slide_increment
            feet.position[2 + 3*feet_set[2]] = 0
            feet.position[0 + 3*feet_set[3]] -= slide_increment
            feet.position[2 + 3*feet_set[3]] = 0
            move()

def trot(stride_length, stride_steer, stride_height):
    feet_position = feet.position
    lean_increments = 20
    step_increments = 12
    slide_increment = (2*stride_length)/step_increments
    
    # ADJUST LEAN COEFFICIENTS SO THAT IT, WELL, WORKS (the 0.25 and the 0.5) --> INSTEAD OF USING ABSOLUTE INCREMENTS, MAKE THEM BASED ON DISTANCE TO TRAVEL (distance/interval distance)
    lean = (0.5*stride_length*cos(stride_steer) - stride.cg_x_offset, 0.25*stride_length*sin(stride_steer) - stride.cg_y_offset)
    
    # Lean in the correct direction for the steps
    if body.position[0] != lean[0] or body.position[1] != lean[1]:
        
        lean_start = (body.position[0], body.position[1])
        
        for j in range(lean_increments + 1):
            body.position[0] = (j/lean_increments)*(lean[0] - lean_start[0])
            body.position[1] = (j/lean_increments)*(lean[1] - lean_start[1])
            move()
    
    
    for i in range(2):
        
        if i == 0:
            feet_set = (1, 2, 0, 3)  # Determines which feet are lifting and which are sliding (lift, lift, slide, slide)
            
        if i == 1:
            feet_set = (3, 0, 1, 2)
            
        # Assign target points for step
        foot_start = (feet_position[0 + 3*feet_set[0]], feet_position[1 + 3*feet_set[0]])
        foot_end = stride_length
        
        # leg move phase
        for j in range(step_increments):
            t = (j/(step_increments - 1))  # Point along curve, from 0 to 1
        
            # Move the feet that are lifting this cycle
            feet_position[0 + 3*feet_set[0]] = (foot_start[0]*(1-t)**3 + (3*foot_start[0])*t*(1-t)**2 + 6*foot_end*(1-t)*t**2 + foot_end*t**3)*cos(stride_steer)
            feet_position[1 + 3*feet_set[0]] = (foot_start[1]*(1-t)**3 + (3*foot_start[1])*t*(1-t)**2 + 6*foot_end*(1-t)*t**2 + foot_end*t**3)*sin(stride_steer) - feet.walk_lateral
            feet_position[2 + 3*feet_set[0]] = (9/4)*stride_height*t*(1-t)**2 + 3*stride_height*(1-t)*t**2
            feet_position[0 + 3*feet_set[1]] = (foot_start[0]*(1-t)**3 + (3*foot_start[0])*t*(1-t)**2 + 6*foot_end*(1-t)*t**2 + foot_end*t**3)*cos(stride_steer)
            feet_position[1 + 3*feet_set[1]] = (foot_start[1]*(1-t)**3 + (3*foot_start[1])*t*(1-t)**2 + 6*foot_end*(1-t)*t**2 + foot_end*t**3)*sin(stride_steer) + feet.walk_lateral
            feet_position[2 + 3*feet_set[1]] = (9/4)*stride_height*t*(1-t)**2 + 3*stride_height*(1-t)*t**2
            
            # Move the feet that are sliding this cycle - MAKE THIS A LOOP THAT ONLY MOVES FEET THAT HAVE ALREADY LIFTED
            feet_position[0 + 3*feet_set[2]] -= slide_increment*cos(stride_steer)
            feet_position[1 + 3*feet_set[2]] -= slide_increment*sin(stride_steer)
            feet_position[2 + 3*feet_set[2]] = 0
            feet_position[0 + 3*feet_set[3]] -= slide_increment*cos(stride_steer)
            feet_position[1 + 3*feet_set[3]] -= slide_increment*sin(stride_steer)
            feet_position[2 + 3*feet_set[3]] = 0
            
            feet.position = feet_position
            move()


def waddle():
    return


def gallop():
    return


# Startup stuff
body.position = body.walk_position
feet.position = feet.walk_position
move()

stride.steer = np.deg2rad(0)
stride.length = 0 # INTEGRATE THIS INTO A "SPEED" PARAMETER -> SCALE THE INTERVAL AND/OR TIMING TOO, NOT JUST THE LENGTH?

# Pause for a second so I can frantically grab the robot after hitting start
time.sleep(1.5)

# TESTING BITS -----------------------------------------------------
# Define a function for the move thread to run
def read_imu():
	# Read acceleration & magnetometer.
	accel_x, accel_y, accel_z = mag_accel.accelerometer
	mag_x, mag_y, mag_z = mag_accel.magnetometer
	gyro_x, gyro_y, gyro_z = gyro.gyroscope

	return accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z

def move_thread_f():
    while True:
        #thread_lock.acquire()
        trot(stride.length, stride.steer, stride.height)
        #thread_lock.release()
        time.sleep(0.025)

def imu_thread_f():
    while True:
        accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z = read_imu()
        newTime = time.time()
        dt = newTime - imu.currTime
        imu.currTime = newTime
        
        sensorfusion.computeAndUpdateRollPitchYaw(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, dt)
        
        if sensorfusion.roll > 5 or sensorfusion.pitch > 5:
        
            # PID Bits
            imu.pitch_error = imu.target_pitch - sensorfusion.roll
            imu.roll_error = imu.target_roll - sensorfusion.pitch
        
            #thread_lock.acquire()
            body.position[3] -= imu.pitch_error*imu.balance_P + imu.pitch_prev_error*imu.balance_D + imu.pitch_sum_error*imu.balance_I
            body.position[4] += imu.roll_error*imu.balance_P + imu.roll_prev_error*imu.balance_D + imu.roll_sum_error*imu.balance_I
            #thread_lock.release()
        
        
            imu.pitch_prev_error = imu.pitch_error
            imu.roll_prev_error = imu.roll_error
            imu.pitch_sum_error += imu.pitch_error
            imu.roll_sum_error += imu.roll_error
        
        time.sleep(0.1)


# Create a lock to prevent simultaneous access of variables
thread_lock = threading.Lock()

# Create threads to manage the various important operational bits
step_thread = threading.Thread(target=move_thread_f)
orientation_thread = threading.Thread(target=imu_thread_f)

#-------------------------------------------------------------------

try:
    step_thread.start()
    orientation_thread.start()
    
    orientation_thread.join()
    step_thread.join()

except KeyboardInterrupt:
    reset_pose()
    
    mylcd.lcd_clear()
    mylcd.lcd_display_string("Shutting", 1, 4)
    mylcd.lcd_display_string("Down", 2, 6)
    time.sleep(1)
    mylcd.lcd_clear()
    mylcd.backlight(0)
    
    pass