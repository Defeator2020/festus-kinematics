import time
import numpy as np
import math
from adafruit_servokit import ServoKit

# Define the servo controller board and its parameters
kit = ServoKit(channels=16)
for i in range(4, 16):
    kit.servo[i].set_pulse_width_range(1000, 2000)
    kit.servo[i].actuation_range = 120

# Define the size of various system elements (in mm)
body_length = 90  # Halfway between front to rear leg pivots
body_width = 40  # Halfway between hip pivots
hip_offset = 56
upper_leg = 108
lower_leg = 133

# Define the servo rest offsets (the amount to adjust from 0 to make all elements point straight down)
servo_offsets = [0, 0, 0, 0, -74, 76, -78, 78, 24, -24, 28, -28, 11, -2, -6, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips

# Define an array to hold all of the servo values, to be written simultaneously. Populated initially with rest/default positions
servo_positions = [90, 90, 90, 90, 0, 120, 0, 120, 60, 60, 60, 60, 60, 60, 60, 60]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips

# Define values for flipping the servo angles
flip = [1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips

# KINEMATIC CALCULATIONS
def leg_angles(feet_targets):
    
    foot_targets = [0]*12
    
    foot_targets[0] = 0
    foot_targets[1] = 0
    foot_targets[2] = 180
    foot_targets[3] = 0
    foot_targets[4] = 0
    foot_targets[5] = 180
    foot_targets[6] = 0
    foot_targets[7] = 0
    foot_targets[8] = 180
    foot_targets[9] = 0
    foot_targets[10] = 0
    foot_targets[11] = 180
    
    for leg in range(4):
    
        # Y offset calculation --------------------
        adjusted_z = math.sqrt(foot_targets[2 + 3*leg]**2 + foot_targets[1 + 3*leg]**2 - hip_offset**2)
        hip_angle = np.rad2deg(math.atan(foot_targets[1 + 3*leg]/foot_targets[2 + 3*leg]) + math.atan(adjusted_z/hip_offset)) - 90


        # X offset calculation --------------------
        shoulder_offset = np.rad2deg(math.atan(foot_targets[0 + 3*leg]/adjusted_z))
        leg_length = math.sqrt(adjusted_z**2 + foot_targets[0 + 3*leg]**2)


        # Z offset calculation --------------------
        shoulder_base_angle = np.rad2deg(math.acos((upper_leg**2 + leg_length**2 - lower_leg**2)/(2*upper_leg*leg_length)))
        wrist_angle = 180 - np.rad2deg(math.acos((lower_leg**2 + upper_leg**2 - leg_length**2)/(2*lower_leg*upper_leg)))
        
        servo_positions[12 + leg] = (hip_angle)*flip[12 + leg] + servo_offsets[12 + leg] + 60
        servo_positions[8 + leg] = (shoulder_base_angle + shoulder_offset)*flip[8 + leg] + servo_offsets[8 + leg] + 60
        servo_positions[4 + leg] = (wrist_angle)*flip[4 + leg] + servo_offsets[4 + leg] + 60


target_positions = [0, 0, 180, 0, 0, 180, 0, 0, 180, 0, 0, 180]
leg_angles(target_positions)

# Write positions to servos
print(servo_positions)  # Print servo angles for debugging
for i in range(16):
    kit.servo[i].angle = servo_positions[i]