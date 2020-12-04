from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

servo_offsets = [0, 0, 0, 0, -78, 80, -86, 86, 33, -32, 36, -36, 11, -2, -6, -5]

for i in range(4, 15):
        kit.servo[i].set_pulse_width_range(1000, 2000)

x = range(15)
for i in x:
	kit.servo[i].angle = 60 + servo_offsets[i]
