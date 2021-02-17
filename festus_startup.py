import time
import threading
import numpy as np
from math import sqrt, sin, cos, acos, atan
from adafruit_servokit import ServoKit
import I2C_LCD_driver

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

# Instantiate various objects
body = Body()
feet = Feet()
servos = Servos()

# Do a neat LCD animation - this takes extra time right now, and should be able to run in parallel with other startup processes (so that eah new dot actually means that something else has been intialized and is green)
mylcd = I2C_LCD_driver.lcd()
mylcd.backlight(1)
mylcd.lcd_display_string("Initializing", 1, 0) # Add dots animation (ellipses) after this while the IMU is initializing
time.sleep(1)
for i in range(4):
    mylcd.lcd_display_string(".", 1, 12 + i)
    time.sleep(0.25)
mylcd.lcd_clear()
mylcd.lcd_display_string("System", 1, 5)
mylcd.lcd_display_string("Online", 2, 5)

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


# Startup stuff
body.position = [-90, 0, 70, 0, 0, 0]#body.lay_position
feet.position = [0, 56, 0, 0, -56, 0, 0, 56, 0, 0, -56, 0]#feet.lay_position

# ADD A BEEP HERE TO ALERT THAT IT IS STANDING UP?

move()

# Stand up - rotate hips so that feet are underneath them, and then rise
hip_distances = [74.34209822423185 - servos.positions[12], 54.65790177576814 - servos.positions[13], 47.65790177576814 - servos.positions[14], 58.34209822423185 - servos.positions[15]]
rotate_increments = 20
hip_steps = [x / rotate_increments for x in hip_distances]

for i in range(rotate_increments):
    for j in range(4):
        servos.positions[j + 12] += hip_steps[j]
    write_to_servos()