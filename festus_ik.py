import time
import numpy as np
import math
from adafruit_servokit import ServoKit

# Define the servo controller board and its parameters
kit = ServoKit(channels=16)
#kit.frequency = 50  # Hz
for i in range(4, 16):
    kit.servo[i].set_pulse_width_range(1000, 2000)
    kit.servo[i].actuation_range = 120

# Define the size of various system elements (in mm)
body_length = 90  # Halfway between front to rear leg pivots
body_width = 40  # Halfway between shoulder pivots
shoulder_offset = 56
upper_leg = 108
lower_leg = 133

# Define the servo rest offsets (the amount to adjust from 0 to make all elements point straight down)
servo_offsets = [0, 0, 0, 0, -78, 80, -86, 86, 33, -32, 36, -36, 11, -2, -6, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl elbows; rr, rl, fr, fl shoulders

# Define an array to hold all of the servo values, to be written simultaneously. Populated initially with rest/default positions
servo_positions = [90, 90, 90, 90, 0, 120, 0, 120, 60, 60, 60, 60, 60, 60, 60, 60]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl elbows; rr, rl, fr, fl shoulders

# KINEMATIC CALCULATIONS
def leg_angles(feet_targets):
    for leg in range(1):
    
        # Y offset calculation --------------------
        shoulder_angle = 0
        adjusted_z = 180
    
        # X offset calculation --------------------
        elbow_offset = 0
        leg_length = 180
    
        # Z offset calculation --------------------
        elbow_base_angle = 0
        wrist_angle = 0
        
        servo_positions[12 + leg] = shoulder_angle + servo_offsets[12 + leg] + 60
        servo_positions[8 + leg] = elbow_base_angle + elbow_offset + servo_offsets[8 + leg] + 60
        servo_positions[4 + leg] = wrist_angle + servo_offsets[4 + leg] + 60
        
#target_positions = [0, 0, 180, 0, 0, 180, 0, 0, 180, 0, 0, 180]
#leg_angles(target_positions)

# Write positions to servos
for i in range(16):
    kit.servo[i].angle = servo_positions[i]
print(servo_positions)  # Print servo angles for debugging