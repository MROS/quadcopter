from motor_module import Motor
import time

# It's seems that raspberry can't identify Chinese(I have not try to adjust), so I use Englist temporarily.
# motor class has limited lots of functions, but You Just need to konw:
# motor::set_speed(s) means set motor's speed to s
# when s more than 5, motor will start to run.
# when four motors all get to 50, quadcopter should start to leave the ground.
# motor::getW is the current speed in this motor
# Once you know these, you can start to write your control process! Such as balance, and all kinds of moving.

class Quad():
    PIN_FL = 22
    PIN_FR = 23
    PIN_BR = 24
    PIN_BL = 25

    def __init__(self):
        self.motors = {'FL': Motor('FL', PIN_FL),
                       'FR': Motor('FR', PIN_FR),
                       'BL': Motor('BL', PIN_BL),
                       'BR': Motor('BR', PIN_BR)}

    def init_motor(self):
        # print('***Disconnect ESC power')
        # print('***then press ENTER')
        res = raw_input()
        for mo in self.motors.values():
            mo.start()
            mo.setW(100)
            #NOTE:the angular motor speed W can vary from 0 (min) to 100 (max)
            #the scaling to pwm is done inside motor class
            # print('***Connect ESC Power')
            # print('***Wait beep-beep')
            # print('***then press ENTER')
            res = raw_input()
            for mo in self.motors.values():
                mo.setW(0)
                # print('***Wait N beep for battery cell')
                # print('***Wait beeeeeep for ready')
                # print('***then press ENTER')
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

