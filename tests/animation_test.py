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
body_length = 180  # Front to rear leg pivots
body_width = 80  # Between shoulder pivots
shoulder_offset = 56
upper_leg = 108
lower_leg = 133

# Define the servo rest offsets (the amount to adjust from 0 to make all elements point straight down)
servo_offsets = [0, 0, 0, 0, -78, 80, -86, 86, 33, -32, 36, -36, 11, -2, -6, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl elbows; rr, rl, fr, fl shoulders

# Define an array to hold all of the servo values, to be written simultaneously. Populated initially with rest/default positions
servo_positions = [90, 90, 90, 90, 0, 120, 0, 120, 60, 60, 60, 60, 60, 60, 60, 60]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl elbows; rr, rl, fr, fl shoulders


# KINEMATIC CALCULATIONS
def leg_angles(target_body_parameters):

    target_x = target_body_parameters[0]
    target_y = target_body_parameters[1]
    target_z = target_body_parameters[2]
    target_yaw = np.deg2rad(target_body_parameters[3])
    target_pitch = np.deg2rad(target_body_parameters[4])
    target_roll = np.deg2rad(target_body_parameters[5])
    
    leg_target_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # x, y, z; rr, rl, fr, fl -> PLACEHOLDER; SHOULD CREATE WHEN ACTUALLY BEING FILLED?
    
    # Yaw calculation --------------------
    shoulder_radius = math.sqrt((body_length/2)**2 + (body_width/2)**2)
    alpha = math.atan(body_width/body_length)
    yaw_lateral_offset = shoulder_radius*math.cos(alpha+target_yaw) - body_length/2
    yaw_longitudinal_offset = shoulder_radius*math.sin(alpha+target_yaw) - body_width/2
    
    # Pitch calculation --------------------
    pitch_height_offset = (body_length*math.sin(target_pitch))/2
    longitudinal_foot_offset = abs((body_length*(1 - math.cos(target_pitch)))/2)
    
    # Roll calculation --------------------
    roll_height_offset = (body_width*math.sin(target_roll))/2
    lateral_foot_offset = abs((body_width*(1 - math.cos(target_roll)))/2)
    
    # Compensate for corner height shifts because of chassis orientation
    leg_target_positions[0] = target_x - longitudinal_foot_offset + yaw_longitudinal_offset
    leg_target_positions[1] = target_y - lateral_foot_offset + yaw_lateral_offset
    leg_target_positions[2] = target_z - pitch_height_offset + roll_height_offset
    leg_target_positions[3] = target_x - longitudinal_foot_offset - yaw_longitudinal_offset
    leg_target_positions[4] = target_y + lateral_foot_offset + yaw_lateral_offset
    leg_target_positions[5] = target_z - pitch_height_offset - roll_height_offset
    leg_target_positions[6] = target_x + longitudinal_foot_offset + yaw_longitudinal_offset
    leg_target_positions[7] = target_y - lateral_foot_offset - yaw_lateral_offset
    leg_target_positions[8] = target_z + pitch_height_offset + roll_height_offset
    leg_target_positions[9] = target_x + longitudinal_foot_offset - yaw_longitudinal_offset
    leg_target_positions[10] = target_y + lateral_foot_offset - yaw_lateral_offset
    leg_target_positions[11] = target_z + pitch_height_offset - roll_height_offset

    for i in range(4):  # Do this for each leg
        # Y offset calculation --------------------
        shoulder_angle = np.rad2deg(math.atan(leg_target_positions[1 + 3*i]/leg_target_positions[2 + 3*i]))
        leg_adjusted_z = math.sqrt(leg_target_positions[2 + 3*i]**2 + leg_target_positions[1 + 3*i]**2 - shoulder_offset**2)


        # X offset calculation --------------------
        elbow_angle_shift = np.rad2deg(math.atan(leg_target_positions[0 + 3*i]/leg_adjusted_z))  # Adjusts the elbow angle calculated in the next part so that the leg is angled forward or backward the correct amount
        leg_target_length = math.sqrt(leg_target_positions[0 + 3*i]**2 + leg_adjusted_z**2)  # Finds the target length of the leg based on the x and z targets


        # Z offset calculation --------------------
        elbow_angle_base = np.rad2deg(math.acos((leg_target_length**2 + upper_leg**2 - lower_leg**2)/(2*leg_target_length*upper_leg)))
        wrist_angle = np.rad2deg(math.acos((upper_leg**2 + lower_leg**2 - leg_target_length**2)/(2*upper_leg*lower_leg)))


        # Adjust values for each leg (different directions for each)
        if i == 0:
            shoulder_flip = 1
            elbow_flip = -1
            wrist_flip = 1
            pitch_offset = -1
        elif i == 1:
            shoulder_flip = 1
            elbow_flip = 1
            wrist_flip = -1
            pitch_offset = 1
        elif i == 2:
            shoulder_flip = -1
            elbow_flip = -1
            wrist_flip = 1
            pitch_offset = -1
        elif i == 3:
            shoulder_flip = -1
            elbow_flip = 1
            wrist_flip = -1
            pitch_offset = 1

        # Push servo values to servo position array
        servo_positions[12+i] = (shoulder_angle + target_body_parameters[5])*shoulder_flip + servo_offsets[12+i] + 60
        servo_positions[8+i] = (elbow_angle_base - elbow_angle_shift)*elbow_flip + pitch_offset*target_body_parameters[4] + servo_offsets[8+i] + 60
        servo_positions[4+i] = (180 - wrist_angle)*wrist_flip + servo_offsets[4+i] + 60


# Write positions to servos
def write_to_servos():
    for i in range(16):
        kit.servo[i].angle = servo_positions[i]

def calculate_write(body_position):
    leg_angles(body_position)

    write_to_servos()

def increment(current_position, target_position, steps):
    x_increment = (target_position[0] - current_position[0])/steps
    y_increment = (target_position[1] - current_position[1])/steps
    z_increment = (target_position[2] - current_position[2])/steps
    yaw_increment = (target_position[3] - current_position[3])/steps
    pitch_increment = (target_position[4] - current_position[4])/steps
    roll_increment = (target_position[5] - current_position[5])/steps
    
    for i in range(steps):
        temp_position = [current_position[0] + x_increment*i, current_position[1] + y_increment*i, current_position[2] + z_increment*i, current_position[3] + yaw_increment*i, current_position[4] + pitch_increment*i, current_position[5] + roll_increment*i]
        calculate_write(temp_position)

# FOR TESTING -> Set target position and orientation for chassis
start_position = [0, 0, 200, 0, 0, 0]  # x, y, z (mm); yaw, pitch, roll (deg)
speed = 15

increment(start_position, [0, 0, 200, 30, 0, 0], speed)
time.sleep(.3)
increment([0, 0, 200, 30, 0, 0], [0, 0, 200, -30, 0, 0], speed)
time.sleep(.3)
increment([0, 0, 200, -30, 0, 0], start_position, speed)
time.sleep(.3)
increment(start_position, [0, 0, 200, 0, 20, 0], speed)
time.sleep(.3)
increment([0, 0, 200, 0, 20, 0], [0, 0, 200, 0, -20, 0], speed)
time.sleep(.3)
increment([0, 0, 200, 0, -20, 0], start_position, speed)
time.sleep(.3)
increment(start_position, [0, 0, 200, 0, 0, 30], speed)
time.sleep(.3)
increment([0, 0, 200, 0, 0, 30], [0, 0, 200, 0, 0, -30], speed)
time.sleep(.3)
increment([0, 0, 200, 0, 0, -30], start_position, speed)
time.sleep(.5)
increment(start_position, [0, 0, 180, 20, 15, 20], speed)
time.sleep(.3)
increment([0, 0, 180, 15, 15, 20], [0, 0, 180, -20, 15, 20], speed)
time.sleep(.3)
increment([0, 0, 180, -20, 15, 20], [0, 0, 180, -20, -15, -20], speed)
time.sleep(.3)
increment([0, 0, 180, -20, -15, -20], [0, 0, 180, 20, -15, -20], speed)
time.sleep(.3)
increment([0, 0, 180, 20, -15, -20], [0, 0, 180, 20, 15, 20], speed)
time.sleep(.3)
increment([0, 0, 180, 20, 15, 20], start_position, speed)