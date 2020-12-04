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


# Bezier curves
#p1 = [-5,0]
#p2 = [-2.5,2.5]
#p3 = [5,5]
#p4 = [5,0]

#walk_x = p1[0]*(1-t)**3 + 3*p2[0]*(1-t)**2 + 3*p3[0]*(1-t)*t**2 + p4[0]*t**3
#walk_z = p1[1]*(1-t)**3 + 3*p2[1]*(1-t)**2 + 3*p3[1]*(1-t)*t**2 + p4[1]*t**3


# KINEMATIC CALCULATIONS
def leg_angles(target_body_parameters, individual_offsets):

    target_x = target_body_parameters[0]  # Body shift forward
    target_y = target_body_parameters[1]  # Body shift right
    target_z = target_body_parameters[2]  # Body shift up
    target_yaw = np.deg2rad(target_body_parameters[3])  # Body yaw clockwise
    target_pitch = np.deg2rad(target_body_parameters[4])  # Body pitch backward
    target_roll = np.deg2rad(target_body_parameters[5])  # Body roll left
    
    leg_target_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # x, y, z; rr, rl, fr, fl -> PLACEHOLDER; SHOULD CREATE WHEN ACTUALLY BEING FILLED?
    
    # Yaw calculation --------------------
    shoulder_radius = math.sqrt((body_length)**2 + (body_width)**2)
    alpha = math.atan(body_width/body_length)
    yaw_lateral_offset = shoulder_radius*math.cos(alpha+target_yaw) - body_length
    yaw_longitudinal_offset = shoulder_radius*math.sin(alpha+target_yaw) - body_width
    
    # Pitch calculation --------------------
    pitch_height_offset = (body_length*math.sin(target_pitch))
    pitch_longitudinal_offset = abs(body_length*(1 - math.cos(target_pitch)))
    
    # Roll calculation --------------------
    roll_height_offset = (body_width*math.sin(target_roll))
    roll_lateral_offset = abs(body_width*(1 - math.cos(target_roll)))
    
    # Compensate for corner height shifts because of chassis orientation
    leg_target_positions[0] = -1*target_x - pitch_longitudinal_offset + yaw_longitudinal_offset + individual_offsets[0] + body_length
    leg_target_positions[1] = target_y - roll_lateral_offset + yaw_lateral_offset + individual_offsets[1] + body_width
    leg_target_positions[2] = target_z - pitch_height_offset + roll_height_offset + individual_offsets[2]
    leg_target_positions[3] = -1*target_x - pitch_longitudinal_offset - yaw_longitudinal_offset + individual_offsets[3] + body_length
    leg_target_positions[4] = target_y + roll_lateral_offset + yaw_lateral_offset + individual_offsets[4] - body_width
    leg_target_positions[5] = target_z - pitch_height_offset - roll_height_offset + individual_offsets[5]
    leg_target_positions[6] = -1*target_x + pitch_longitudinal_offset + yaw_longitudinal_offset + individual_offsets[6] - body_length
    leg_target_positions[7] = target_y - roll_lateral_offset - yaw_lateral_offset + individual_offsets[7] + body_width
    leg_target_positions[8] = target_z + pitch_height_offset + roll_height_offset + individual_offsets[8]
    leg_target_positions[9] = -1*target_x + pitch_longitudinal_offset - yaw_longitudinal_offset + individual_offsets[9] - body_length
    leg_target_positions[10] = target_y + roll_lateral_offset - yaw_lateral_offset + individual_offsets[10] - body_width
    leg_target_positions[11] = target_z + pitch_height_offset - roll_height_offset + individual_offsets[11]

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


# FOR TESTING -> Set rest position and orientation for chassis and feet
body_position = [0, 0, 190, 0, 20, 0]  # x, y, z (mm); yaw, pitch, roll (deg)
foot_positions = [-1*body_length, -1*body_width, 0, -1*body_length, body_width, 0, body_length, -1*body_width, -80, body_length, body_width, 0]  # x, y, z; rr, rl, fr, fl

leg_angles(body_position, foot_positions)

# Print servo angles for debugging
print(servo_positions)

write_to_servos()
