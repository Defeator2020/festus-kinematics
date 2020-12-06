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

# Define various servo properties (offset to make 0 straight down, rest positions, and the flip value to compensate for different servo directions)
servo_offsets = [0, 0, 0, 0, -74, 76, -78, 78, 24, -24, 28, -28, 11, -2, -6, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips (same for all three)
servo_positions = [90, 90, 90, 90, 76, 45, 72, 48, 33, 87, 37, 83, 89, 40, 36, 73]
flip = [1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1]


# KINEMATIC CALCULATIONS
def leg_angles(body_pose, foot_pose):
    
    target_x = body_pose[0]  # Body lean forward
    target_y = body_pose[1]  # Body lean right
    target_z = body_pose[2]  # Body get taller
    target_pitch = np.deg2rad(body_pose[3])  # Pitch back
    target_roll = np.deg2rad(body_pose[4])  # Roll left
    target_yaw = np.deg2rad(body_pose[5])  # Yaw clockwise
    foot_targets = [0]*12
    
    # Pitch calculation --------------------
    pitch_height_shift = body_length*math.sin(target_pitch)
    longitudinal_shift = body_length*(1-math.cos(target_pitch))
    
    # Roll calculation --------------------

    
    
    foot_targets[0] = target_x - longitudinal_shift
    foot_targets[1] = -target_y
    foot_targets[2] = target_z - pitch_height_shift
    foot_targets[3] = target_x - longitudinal_shift
    foot_targets[4] = target_y
    foot_targets[5] = target_z - pitch_height_shift
    foot_targets[6] = target_x + longitudinal_shift
    foot_targets[7] = -target_y
    foot_targets[8] = target_z + pitch_height_shift
    foot_targets[9] = target_x + longitudinal_shift
    foot_targets[10] = target_y
    foot_targets[11] = target_z + pitch_height_shift
    
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
        servo_positions[8 + leg] = (shoulder_base_angle + shoulder_offset + body_pose[3])*flip[8 + leg] + servo_offsets[8 + leg] + 60
        servo_positions[4 + leg] = (wrist_angle)*flip[4 + leg] + servo_offsets[4 + leg] + 60


body_position = [0, 0, 180, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
foot_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # x, y, z; rr, rl, fr, fl
leg_angles(body_position, foot_positions)

# Write positions to servos
print(servo_positions)  # Print servo angles for debugging
for i in range(16):
    kit.servo[i].angle = servo_positions[i]