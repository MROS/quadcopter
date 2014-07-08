from RPIO import PWM

class Motor():
    def __init__(self, name, pin, power_min=0, power_max=1000):
        self.name = name
        self.powered = False
        self.pin = pin
        self.power_min = power_min
        self.power_max = power_max
        self.pulse_width = 1000
        self.servo = PWM.Servo()

    def start(self):
        'Run the procedure to init the PWM'
        # self.servo = PWM.Servo()
        self.powered = True

    def stop(self):
        'Stop PWM signal'
        # self.setW(0)
        self.set_power(0)
        if self.powered:
            self.servo.stop_servo(self.pin)
            self.powered = False

    def set_power(self, power):
        '''
        Update the pulse width of motor
        pulse width = power * 10 + 1000
        '''
        if not self.powered:
            return

        if power < self.power_min:
            power = self.power_min
        elif power > self.power_max:
            power = self.power_max

        self.pulse_width = 1000 + power * 10
        self.servo.set_servo(self.pin, self.pulse_width)

    # def increaseW(self, step=1):
    #     'increases W% for the motor'
    #     self.__W = self.__W + step
    #     self.setW(self.__W)

    # def decreaseW(self, step=1):
    #     'decreases W% for the motor'
    #     self.__W = self.__W - step
    #     self.setW(self.__W)

    # def getW(self):
    #     'retuns current W%'
    #     return self.__W

    # def setW(self, W):
    #     'Checks W% is between limits than sets it'
    #     PW = 0
    #     self.__W = W
    #     if self.__W < self.__WMin:
    #         self.__W = self.__WMin
    #     if self.__W > self.__WMax:
    #         self.__W = self.__WMax
    #     PW = (1000 + (self.__W) * 10)
    #     # Set servo to xxx us
    #     if self.powered:
    #         self.servo.set_servo(self.pin, PW)
