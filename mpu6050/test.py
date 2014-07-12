import mpu6050
import time

mpu6050.initialize()
mpu6050.set_z_gyro_offset(241)

while 1:
	#print(mpu6050.get_yaw_pitch_roll())
	print(mpu6050.get_motion())
	time.sleep(0.1)

