import time
import numpy as np
import math
import signal
from adafruit_servokit import ServoKit
from xbox360controller import Xbox360Controller
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
    radius = math.sqrt(length**2 + width**2)  # Distance from center of body to shoulder (mm)
    alpha = math.atan(width/length)  # Existing angle between +x-axis and radius line about center of body (rad)
    
    # Define some useful positions
    rest_position = [0, 0, 190, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
    lay_position = [-90, 0, 70, 0, 0, 0]  # x, y, z (mm), pitch, roll, yaw (deg)
    
    position = rest_position


class Feet:
    rest_position = [0, 15, 0, 0, -15, 0, 0, 15, 0, 0, -15, 0]  # x, y, z; rr, rl, fr, fl
    #rest_position = [0, 10, 0, 0, -10, 0, 0, 10, 0, 0, -10, 0]  # x, y, z; rr, rl, fr, fl
    lay_position = [0, 15, 0, 0, -15, 0, 0, 15, 0, 0, -15, 0]  # x, y, z; rr, rl, fr, fl
    
    position = rest_position


class Servos:
    # Define various servo properties (offset to make 0 straight down, rest positions, and the flip value to compensate for different servo directions)
    offsets = [0, 0, 0, 0, -74, 76, -78, 78, 24, -24, 28, -28, 11, -2, -6, -5]  # 1, 2, 3, 4 peripheral; rr, rl, fr, fl wrists; rr, rl, fr, fl shoulders; rr, rl, fr, fl hips (same for all three)
    positions = [90, 90, 90, 90, 76, 45, 72, 48, 33, 87, 37, 83, 89, 40, 36, 73]
    flip = [1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1]

class Stride:
    # Define various gait parameters (mm)
    cg_x_offset = -20  # Forward of center
    cg_y_offset = -10  # Right of center
    length = 0  # Midpoint to either extreme of step
    height = 35
    longitudinal_shift = 0  # How far forward the feet are shifted during steps
    static_lean_margin = 50  # Distance past zero moment point to lean in order to maintain stability when lifting one leg or walking in statically stable gait
    
    # Bezier curves
    p1 = [-length + longitudinal_shift,0]
    p2 = [-length/2 + longitudinal_shift, height/2]
    p3 = [1.3*length + longitudinal_shift,1.3*height]
    p4 = [length + longitudinal_shift,0]


# Instantiate various objects
body = Body()
feet = Feet()
servos = Servos()
stride = Stride()
mylcd = I2C_LCD_driver.lcd()

mylcd.backlight(1)
mylcd.lcd_display_string("Initializing", 1, 0) # Add dots animation (ellipses) after this while the IMU is initializing
time.sleep(1)
for i in range(4):
    mylcd.lcd_display_string(".", 1, 12 + i)
    time.sleep(1)
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

def reset_pose():
    # Set rest position and orientation for chassis and feet
    body.position = body.rest_position
    feet.position = feet.rest_position
    move()

def move():
    try:
        leg_angles()
    except:
        print("Math domain error!")
    
    try:
        write_to_servos()
    except:
        print("Angle out of range!")

# Testing function 1 - lift a specific leg
def lift_leg(leg):
    increments = 25
    lift_increments = (stride.height - feet.position[2 + 3*leg])/25
    
    for i in range(increments):
        feet.position[2 + 3*leg] += lift_increments
        move()
        
    for i in range(increments):
        feet.position[2 + 3*leg] -= lift_increments
        move()

def on_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))
    if button.name == "button_y":
        body.position = body.rest_position
        feet.position = feet.rest_position
    
    if button.name == "button_trigger_l":
        body.position[5] -= 5
    
    if button.name == "button_trigger_r":
        body.position[5] += 5
    
    move()

def on_button_released(button):
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    if axis.name == "axis_l":
        body.position[0] = -axis.y * 75
        body.position[1] = axis.x * 75
    
    if axis.name == "axis_r":
        body.position[3] = axis.y * 50
        body.position[4] = -axis.x * 50
    
    if axis.name == "hat":
        if axis.x == 1:
            lift_leg(2)
        elif axis.x == -1:
            lift_leg(1)
        elif axis.y == 1:
            lift_leg(3)
        elif axis.y == -1:
            lift_leg(0)
    move()

# Startup stuff
reset_pose()
with Xbox360Controller() as controller:
    controller.set_led(Xbox360Controller.LED_BLINK_SLOW)

try:
    with Xbox360Controller(0, axis_threshold=0) as controller:
        # Button events
        controller.button_a.when_pressed = on_button_pressed
        controller.button_y.when_pressed = on_button_pressed
        controller.button_trigger_l.when_pressed = on_button_pressed
        controller.button_trigger_r.when_pressed = on_button_pressed
        
        controller.button_a.when_released = on_button_released
        controller.button_y.when_released = on_button_released
        controller.button_trigger_l.when_released = on_button_released
        controller.button_trigger_r.when_released = on_button_released

        # Axis move event
        controller.axis_l.when_moved = on_axis_moved
        controller.axis_r.when_moved = on_axis_moved
        controller.hat.when_moved = on_axis_moved
        
        signal.pause()

except KeyboardInterrupt:
    reset_pose()
    
    with Xbox360Controller() as controller:
        controller.set_led(Xbox360Controller.LED_TOP_LEFT_ON)
    
    mylcd.lcd_clear()
    mylcd.lcd_display_string("Shutting", 1, 4)
    mylcd.lcd_display_string("Down", 2, 6)
    time.sleep(1)
    mylcd.lcd_clear()
    mylcd.backlight(0)
    pass