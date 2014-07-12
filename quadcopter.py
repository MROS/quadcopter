import mpu6050.mpu6050 as mpu6050
from motor import motor
import time
import math

def radius_to_angle(x):
	return x * 180 / math.pi

# It's seems that raspberry can't identify Chinese(I have not try to adjust), so I use Englist temporarily.
# motor class has limited lots of functions, but You Just need to konw:
# motor::set_speed(s) means set motor's speed to s
				# when s more than 5, motor will start to run.
				# when four motors all get to 50, quadcopter should start to leave the ground.
# motor::getW is the current speed in this motor
# Once you know these, you can start to write your control process! Such as balance, and all kinds of moving.
#!/usr/bin/python

# while True:
	#print('q %f\t%f\t%f\t%f' % mpu6050.get_quaternion())
	#print('e %f\t%f\t%f' % mpu6050.get_euler())
	# ypr = mpu6050.get_yaw_pitch_roll()
	# print(ypr[0] * 180 / 3.14159, "\t", ypr[1] * 180 / 3.14159, "\t", ypr[2] * 180 / 3.14159)
	#print('y %f\t%f\t%f' % mpu6050.get_yaw_pitch_roll())
	#print('a %i\t%i\t%i' % mpu6050.get_linear_accel())
	#print('w %i\t%i\t%i' % mpu6050.get_linear_accel_in_world()))

class quadcopter(object):
	def __init__(self):
		# setting motors
		self.motor_standard = 35
		self.motors = {'left' : motor('left', 25, simulation=False),
				'right' : motor('right', 23, simulation=False),
				'rear' : motor('rear', 24, simulation=False),
				'front' : motor('front', 22, simulation=False) }
	def init_hardware(self):
		# init IMU
		mpu6050.initialize()
		mpu6050.set_z_gyro_offset(239);

		# init motors
		print('***Disconnect ESC power')
		print('***then press ENTER')
		res = raw_input()
		for mo in self.motors.values():
			mo.start()
			mo.setW(100)
		#NOTE:the angular motor speed W can vary from 0 (min) to 100 (max)
		#the scaling to pwm is done inside motor class
		print('***Connect ESC Power')
		print('***Wait beep-beep')
		print('***then press ENTER')
		res = raw_input()
		for mo in self.motors.values():
			mo.setW(0)
		print('***Wait N beep for battery cell')
		print('***Wait beeeeeep for ready')
		print('***then press ENTER')
		res = raw_input()

	def set_all_to(self, s):
		for mo in self.motors.values():
			mo.set_speed(s)
	def stop(self):
		for mo in self.motors.values():
			mo.set_speed(0)

	def set_unique_to(self, mo, s):
		self.motors[mo].set_speed(s)

	def take_off(self):
		self.set_all_to(50)

	def roll_balance(self):
		self.set_unique_to('left', self.motor_standard)
		self.set_unique_to('right', self.motor_standard)
		while True:
			(tmp1, tmp2, roll) = mpu6050.get_yaw_pitch_roll()
			error = int(radius_to_angle(roll) * 0.2)
			left_speed = self.motors['left'].getW() + error
			right_speed = self.motors['right'].getW() - error
			self.set_unique_to('left', left_speed)
			self.set_unique_to('right', right_speed)

	def balance(self):
		# TODO: make quadcopter balance(stop in the air), wait for the study of IMU
		pass

	def descend(self):
		# TODO: It should have different behavior in different height and speed, wait for the study of IMU
		s = max([m.getW() for m in self.motors.values()])
		print(s)
		self.set_all_to(s)
		for w in range(s, 1, -1):
			self.set_all_to(w)
			time.sleep(0.5)
