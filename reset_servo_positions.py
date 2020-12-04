from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

servo_offsets = [0, 0, 0, 0, -90, 90, -90, 90, 0, -4, 2, 0, 12, -5, -9, -2]

for i in range(4, 15):
        kit.servo[i].set_pulse_width_range(750, 2250)

x = range(15)
for i in x:
	kit.servo[i].angle = 90 + servo_offsets[i]
