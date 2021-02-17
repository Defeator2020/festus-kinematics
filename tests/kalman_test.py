import os
import sys
import time
import smbus
import busio
import board
import numpy as np

import adafruit_fxos8700
import adafruit_fxas21002c
from imusensor.filters import kalman 

# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
mag_accel = adafruit_fxos8700.FXOS8700(i2c)
gyro = adafruit_fxas21002c.FXAS21002C(i2c)

sensorfusion = kalman.Kalman()

def read():
	# Read acceleration & magnetometer.
	accel_x, accel_y, accel_z = mag_accel.accelerometer
	mag_x, mag_y, mag_z = mag_accel.magnetometer
	gyro_x, gyro_y, gyro_z = gyro.gyroscope

	return accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z

# accel_x, accel_y, accel_z, mag_x, mag_y, mag_z = read()

count = 0
currTime = time.time()
while True:
	accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z = read()
	newTime = time.time()
	dt = newTime - currTime
	currTime = newTime

	sensorfusion.computeAndUpdateRollPitchYaw(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, dt)

	print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))

	time.sleep(0.01)
