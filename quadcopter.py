import mpu6050.mpu6050 as mpu6050
import sys
from motor import motor
import time
import math
import config

YAW = 0
PITCH = 1
ROLL = 2

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

class PIDcontrol:
	def __init__(self, _kp, _ki, _kd, _pwMax, _pwMin):
		assert _pwMax > _pwMin , 'pwMax should larger than pwMin'
		self.kp = _kp
		self.ki = _kd
		self.kd = _kd
		self.pwMax = _pwMax
		self.pwMin = _pwMin
		self.tp = time.time() * 1000
		self.pError = 0
		self.I_sum = 0
	def compute(self, mError):
		tn = time.time() * 1000
		dt = tn - self.tp
		self.tp = tn

		p = self.kp * mError
		i = self.I_sum + self.ki * mError * dt / 1000
		d = self.kd * (mError - self.pError) * 1000 / dt
		u = p + d + i

		sys.stderr.write("mError=%f, p=%f, i=%f, d=%f" % (mError, p, i, d))

		self.I_sum = i

		if u > self.pwMax:
			u = self.pwMax
		elif u < self.pwMin:
			u = self.pwMin

		return {'p': p, 'i': i, 'd': d, 'u': u}

class balance_info:
	def __init__(self, angle, rate, desired_rate_pidu, motors_speed):
		self.angle = angle
		self.rate = rate
		self.desired_rate_pidu = desired_rate_pidu
		self.motors_speed = motors_speed

class quadcopter(object):
	def __init__(self):
		# setting motors
		self.rate_pid = []
		self.motor_pid = []
		for i in range(0, 3):
			self.rate_pid.append(PIDcontrol(config.ANGLE_KP, config.ANGLE_KI, config.ANGLE_KD, config.ANGLE_MAX, config.ANGLE_MIN))
			self.motor_pid.append(PIDcontrol(config.RATE_KP, config.RATE_KI, config.RATE_KD, config.RATE_MAX, config.RATE_MIN))

		self.motor_standard = 10
		self.angle_standard = [0,3,0]
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
		# res = raw_input()
		time.sleep(2)
		for mo in self.motors.values():
			mo.start()
			mo.setW(100)
		#NOTE:the angular motor speed W can vary from 0 (min) to 100 (max)
		#the scaling to pwm is done inside motor class
		print('***Connect ESC Power')
		print('***Wait beep-beep')
		print('***then press ENTER')
		# res = raw_input()
		time.sleep(2)
		for mo in self.motors.values():
			mo.setW(0)
		print('***Wait N beep for battery cell')
		print('***Wait beeeeeep for ready')
		print('***then press ENTER')
		time.sleep(2)

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

	def keep_balance(self):
		while True:
			self.a_balance()
			
	def a_balance(self):
		
		# angle originally arranged by [yaw, pitch, roll]
		# rate originally arranged by [roll, pitch, yaw]
		angle = list(mpu6050.get_yaw_pitch_roll())
		angle = map(lambda x: x * 180 / math.pi, angle)
		rate = list(mpu6050.get_motion())[3:6]
		rate = map(lambda x: -x / 131.0, rate)
		
		rate[0], rate[2] = rate[2], rate[0]
		# now rate and angle are all arranged by [yaw, pitch, roll]
		
		
		#	roll = roll * 180 / math.pi;
		#	pitch = pitch * 180 / math.pi;
		#	roll_s = -roll_s
		#	pitch_s = -pitch_s
		#	roll_s = roll_s / 131.0
		#	pitch_s = pitch_s / 131.0

		#if roll != roll or pitch != pitch:
		#	continue
		
		desired_rate_pidu = []
		for i in range(0, 3):
			desired_rate_pidu.append(self.rate_pid[i].compute(angle[i] - self.angle_standard[i]))
		
		desired_rate = [x['u'] for x in desired_rate_pidu]
		
		desired_motor_pidu = []
		for i in range(0, 3):
			desired_motor_pidu.append(self.motor_pid[i].compute(desired_rate[i] - rate[i]))
		
		desired_motor = [int(x['u']) for x in desired_motor_pidu]

		self.set_unique_to('left', self.motor_standard + desired_motor[ROLL])
		self.set_unique_to('right', self.motor_standard - desired_motor[ROLL])
		self.set_unique_to('front', self.motor_standard + desired_motor[PITCH])
		self.set_unique_to('rear', self.motor_standard - desired_motor[PITCH])
		
		motor_speed = {}
		for (name, motor) in self.motors.items():
			motor_speed[name] = motor.getW()

		sys.stderr.write("pitch: %f, roll: %f" % (angle[PITCH], angle[ROLL]))
		sys.stderr.write("desired roll rate: %f, really roll rate: %f" % (desired_rate[ROLL], rate[ROLL]))
		sys.stderr.write("desired pitch rate: %f, really pitch rate: %f" % (desired_rate[PITCH], rate[PITCH]))
		sys.stderr.write("desired_roll_motor: %d" % desired_motor[ROLL])
		sys.stderr.write("desired_pitch_motor: %d" % desired_motor[PITCH])
		
		
		return balance_info(angle, rate, desired_rate_pidu, motor_speed)


	def descend(self):
		# TODO: It should have different behavior in different height and speed, wait for the study of IMU
		s = max([m.getW() for m in self.motors.values()])
		print(s)
		self.set_all_to(s)
		for w in range(s, 1, -1):
			self.set_all_to(w)
			time.sleep(0.5)

