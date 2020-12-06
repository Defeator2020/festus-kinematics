from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
for i in range(4, 16):
    kit.servo[i].set_pulse_width_range(1000, 2000)
    kit.servo[i].actuation_range = 120

servo_offsets = [0, 0, 0, 0, -74, 76, -78, 78, 24, -24, 28, -28, 11, -2, -6, -5]
#servo_offsets = [0, 0, 0, 0, -78, 80, -86, 86, 24, -24, 28, -28, 11, -2, -6, -5]

x = range(15)
for i in x:
	kit.servo[i].angle = 60 + servo_offsets[i]
