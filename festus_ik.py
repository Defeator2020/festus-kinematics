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
radius = math.sqrt(body_length**2 + body_width**2)  # Distance from center of body to shoulder (mm)
alpha = math.atan(body_width/body_length)  # Existing angle between +x-axis and radius line about center of body (rad)

# Define various gait parameters
cg_x_offset = -10  # Forward of center
cg_y_offset = -10  # Right of center
leg_lift_height = 50
static_lean_margin = 25  # Distance past zero moment point to lean (mm) in order to maintain stability when lifting one leg or walking in statically stable gait

# Define various servo properties (offset to make 0 straight down, rest positions, and the flip value to compensate for different servo directions)
servo_offsets = [0, 0, 0, 0, -74, 76, -78, 78, 24, -24, 28, -28, 11, -2, -6, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips (same for all three)
servo_positions = [90, 90, 90, 90, 76, 45, 72, 48, 33, 87, 37, 83, 89, 40, 36, 73]
flip = [1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1]

# Define some useful positions
rest_body_position = [0, 0, 190, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
rest_foot_positions = [0, 10, 0, 0, -10, 0, 0, 10, 0, 0, -10, 0]  # x, y, z; rr, rl, fr, fl

lay_body_position = [-90, 0, 70, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
lay_foot_positions = [0, 15, 0, 0, -15, 0, 0, 15, 0, 0, -15, 0]  # x, y, z; rr, rl, fr, fl

body_position = rest_body_position
foot_positions = rest_foot_positions


# KINEMATIC CALCULATIONS
def leg_angles(body_pose, foot_pose):
    global servo_positions
    
    target_x = body_pose[0]  # Body lean forward
    target_y = body_pose[1]  # Body lean right
    target_z = body_pose[2]  # Body get taller
    target_pitch = np.deg2rad(body_pose[3])  # Pitch back
    target_roll = np.deg2rad(body_pose[4])  # Roll left
    target_yaw = np.deg2rad(body_pose[5])  # Yaw clockwise
    foot_targets = [0]*12
    
    # Pitch calculation --------------------
    pitch_height_shift = body_length*math.sin(target_pitch)
    longitudinal_pitch = body_length*(1-math.cos(target_pitch))
    
    # Roll calculation --------------------
    roll_height_shift = body_width*math.sin(target_roll)
    lateral_roll = body_width*(1-math.cos(target_roll))
    
    # Yaw calculation --------------------
    longitudinal_yaw = radius*math.cos(alpha - target_yaw) - body_length
    lateral_yaw = radius*math.sin(alpha - target_yaw) - body_width
    
    
    foot_targets[0] = target_x - foot_pose[0] - longitudinal_pitch - longitudinal_yaw
    foot_targets[1] = -target_y + foot_pose[1] + lateral_roll - lateral_yaw
    foot_targets[2] = target_z - foot_pose[2] - pitch_height_shift + roll_height_shift
    foot_targets[3] = target_x - foot_pose[3] - longitudinal_pitch - longitudinal_yaw
    foot_targets[4] = target_y - foot_pose[4] + lateral_roll + lateral_yaw
    foot_targets[5] = target_z - foot_pose[5] - pitch_height_shift - roll_height_shift
    foot_targets[6] = target_x - foot_pose[6] + longitudinal_pitch + longitudinal_yaw
    foot_targets[7] = -target_y + foot_pose[7] + lateral_roll + lateral_yaw
    foot_targets[8] = target_z - foot_pose[8] + pitch_height_shift + roll_height_shift
    foot_targets[9] = target_x - foot_pose[9] + longitudinal_pitch + longitudinal_yaw
    foot_targets[10] = target_y - foot_pose[10] + lateral_roll - lateral_yaw
    foot_targets[11] = target_z - foot_pose[11] + pitch_height_shift - roll_height_shift
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
        
        servo_positions[12 + leg] = (hip_angle + ((-1)**(leg+1))*body_pose[4])*flip[12 + leg] + servo_offsets[12 + leg] + 60
        servo_positions[8 + leg] = (shoulder_base_angle + shoulder_offset + body_pose[3])*flip[8 + leg] + servo_offsets[8 + leg] + 60
        servo_positions[4 + leg] = (wrist_angle)*flip[4 + leg] + servo_offsets[4 + leg] + 60

# Write positions to servos
def write_to_servos():
    print(servo_positions)  # Print servo angles for debugging
    for i in range(16):
        kit.servo[i].angle = servo_positions[i]

def reset_pose():
    global body_position
    global foot_positions
    
    # Set rest position and orientation for chassis and feet
    body_position = rest_body_position
    foot_positions = rest_foot_positions
    move(body_position, foot_positions)
    
def lift_leg(leg):
    global body_position
    global foot_positions
    
    body_position[0] = -cg_x_offset
    
    if leg == 0 or leg == 2:
        body_position[1] -= (static_lean_margin + cg_y_offset)
    elif leg == 1 or leg == 3:
        body_position[1] += (static_lean_margin - cg_y_offset)
    
    move(body_position, foot_positions)
    time.sleep(.2)
    
    foot_positions[0 + 3*leg] += 0
    foot_positions[1 + 3*leg] += 0
    foot_positions[2 + 3*leg] += leg_lift_height
    move(body_position, foot_positions)

def move(body, feet):
    leg_angles(body, feet)
    write_to_servos()

reset_pose()
time.sleep(1)

move(lay_body_position, lay_foot_positions)
"""
lift_leg(0)
time.sleep(1)

reset_pose()
time.sleep(1)

lift_leg(1)
time.sleep(1)

reset_pose()
time.sleep(1)

lift_leg(2)
time.sleep(1)

reset_pose()
time.sleep(1)

lift_leg(3)
time.sleep(1)

reset_pose()"""