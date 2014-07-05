from motor import motor

# It's seems that raspberry can't identify Chinese(I have not try to adjust), so I use Englist temporarily.
# motor class has limited lots of functions, but You Just need to konw:
# motor::set_speed(s) means set motor's speed to s
				# when s more than 5, motor will start to run.
				# when four motors all get to 50, quadcopter should start to leave the ground.
# Once you know this, you can start to write your control process! Such as balance, and all kinds of moving.

class quadcopter(object):
	def __init__(self):
		self.motors = 
		{'left' : motor('left', 17, sumulation=False),
		 'right' : motor('right', 21, sumulation=False),
		 'rear' : motor('rear', 23, sumulation=False),
		 'front' : motor('front', 24, sumulation=False) }
	def init_motor(self):
		print('***Disconnect ESC power')
		print('***then press ENTER')
		res = raw_input()
		for (name, mo) in self.motors:
			mo.start()
			mo.setW(100)
		#NOTE:the angular motor speed W can vary from 0 (min) to 100 (max)
		#the scaling to pwm is done inside motor class
		print('***Connect ESC Power')
		print('***Wait beep-beep')
		print('***then press ENTER')
		res = raw_input()
		mymotor.setW(0)

	def take_off(self):
		for (name, mo) in self.motors:
			mo.set_speed(50)


	def up(self):
