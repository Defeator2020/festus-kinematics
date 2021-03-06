import time
import numpy as np
import math
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
    radius = math.sqrt(length**2 + width**2)  # Distance from center of body to shoulder (mm)
    alpha = math.atan(width/length)  # Existing angle between +x-axis and radius line about center of body (rad)
    
    # Define some useful positions
    rest_position = [0, 0, 190, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
    lay_position = [-90, 0, 70, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
    walk_position = [0, 0, 190, 0, 0, 0]  # DEBUG
    
    position = rest_position

class Feet:
    walk_lateral = 56
    
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
    cg_x_offset = -12  # Forward of center
    cg_y_offset = -10  # Right of center
    length = 40  # Distance from midpoint to either extreme of step
    height = 70  # Distance from ground to highest control point of Bezier curve
    single_margin = 40  # How far from one side the chassis stays during a single step lean
    steer = np.deg2rad(0)  # Target angle to walk at (clockwise from forward=0) (rad)
    

# Instantiate various objects
body = Body()
feet = Feet()
servos = Servos()
stride = Stride()
mylcd = I2C_LCD_driver.lcd()
sensorfusion = kalman.Kalman()

# KINEMATIC CALCULATIONS
def leg_angles():
    target_x = body.position[0]  # Body lean forward
    target_y = body.position[1]  # Body lean right
    target_z = body.position[2]  # Body get taller
    target_pitch = np.deg2rad(body.position[3])  # Pitch back
    target_roll = np.deg2rad(body.position[4])  # Roll left
    target_yaw = np.deg2rad(body.position[5])  # Yaw clockwise
    foot_targets = [0]*12
    
    # Pitch calculation --------------------
    pitch_height_shift = body.length*math.sin(target_pitch)
    longitudinal_pitch = body.length*(1-math.cos(target_pitch))
    
    # Roll calculation --------------------
    roll_height_shift = body.width*math.sin(target_roll)
    lateral_roll = body.width*(1-math.cos(target_roll))
    
    # Yaw calculation --------------------
    longitudinal_yaw = body.radius*math.cos(body.alpha - target_yaw) - body.length
    lateral_yaw = body.radius*math.sin(body.alpha - target_yaw) - body.width
    
    
    foot_targets[0] = target_x - feet.position[0] - longitudinal_pitch - longitudinal_yaw
    foot_targets[1] = -target_y + feet.position[1] + lateral_roll - lateral_yaw
    foot_targets[2] = target_z - feet.position[2] - pitch_height_shift + roll_height_shift
    foot_targets[3] = target_x - feet.position[3] - longitudinal_pitch - longitudinal_yaw
    foot_targets[4] = target_y - feet.position[4] + lateral_roll + lateral_yaw
    foot_targets[5] = target_z - feet.position[5] - pitch_height_shift - roll_height_shift
    foot_targets[6] = target_x - feet.position[6] + longitudinal_pitch + longitudinal_yaw
    foot_targets[7] = -target_y + feet.position[7] + lateral_roll + lateral_yaw
    foot_targets[8] = target_z - feet.position[8] + pitch_height_shift + roll_height_shift
    foot_targets[9] = target_x - feet.position[9] + longitudinal_pitch + longitudinal_yaw
    foot_targets[10] = target_y - feet.position[10] + lateral_roll - lateral_yaw
    foot_targets[11] = target_z - feet.position[11] + pitch_height_shift - roll_height_shift
    
    for leg in range(4):
        # Y offset calculation --------------------
        adjusted_z = math.sqrt(foot_targets[2 + 3*leg]**2 + foot_targets[1 + 3*leg]**2 - body.hip_offset**2)
        hip_angle = np.rad2deg(math.atan(foot_targets[1 + 3*leg]/foot_targets[2 + 3*leg]) + math.atan(adjusted_z/body.hip_offset)) - 90
        
        # X offset calculation --------------------
        shoulder_offset = np.rad2deg(math.atan(foot_targets[0 + 3*leg]/adjusted_z))
        leg_length = math.sqrt(adjusted_z**2 + foot_targets[0 + 3*leg]**2)
        
        # Z offset calculation --------------------
        shoulder_base_angle = np.rad2deg(math.acos((body.upper_leg**2 + leg_length**2 - body.lower_leg**2)/(2*body.upper_leg*leg_length)))
        wrist_angle = 180 - np.rad2deg(math.acos((body.lower_leg**2 + body.upper_leg**2 - leg_length**2)/(2*body.lower_leg*body.upper_leg)))
        
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

# Startup stuff
body.position = body.walk_position
feet.position = feet.walk_position
move()

stride.steer = np.deg2rad(0)
stride.length = 40 # INTEGRATE THIS INTO A "SPEED" PARAMETER -> SCALE THE INTERVAL AND/OR TIMING TOO, NOT JUST THE LENGTH?

time.sleep(1)

# THE BIT BEING TESTED --------------------
def read_imu():
	# Read acceleration & magnetometer.
	accel_x, accel_y, accel_z = mag_accel.accelerometer
	mag_x, mag_y, mag_z = mag_accel.magnetometer
	gyro_x, gyro_y, gyro_z = gyro.gyroscope

	return accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z

# count = 0
balance_P = 0.1
balance_D = balance_P/4
balance_I = balance_D/10

pitch_prev_error = 0
roll_prev_error = 0
pitch_sum_error = 0
roll_sum_error = 0
target_pitch = -6
target_roll = 0
currTime = time.time()
try:
    while True:
        accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z = read_imu()
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime
        
        sensorfusion.computeAndUpdateRollPitchYaw(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, dt)
        
        # PID Bits
        pitch_error = target_pitch - sensorfusion.roll
        roll_error = target_roll - sensorfusion.pitch
        
        body.position[3] -= pitch_error*balance_P + pitch_prev_error*balance_D + pitch_sum_error*balance_I
        body.position[4] += roll_error*balance_P + roll_prev_error*balance_D + roll_sum_error*balance_I
        move()
        
        pitch_prev_error = pitch_error
        roll_prev_error = roll_error
        pitch_sum_error += pitch_error
        roll_sum_error += roll_error

except KeyboardInterrupt:
    reset_pose()
    
    mylcd.lcd_clear()
    mylcd.lcd_display_string("Shutting", 1, 4)
    mylcd.lcd_display_string("Down", 2, 6)
    time.sleep(1)
    mylcd.lcd_clear()
    mylcd.backlight(0)
    
    pass