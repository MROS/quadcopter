#solenero.tech@gmail.com
#solenerotech.wordpress.com

#solenerotech 2013.09.06

from motor import motor
import time

mymotor = motor('m1', 22, simulation=False)
#where 17 is  GPIO17 = pin 11

print('***Disconnect ESC power')
print('***then press ENTER')
res = raw_input()
mymotor.start()
mymotor.setW(100)

#NOTE:the angular motor speed W can vary from 0 (min) to 100 (max)
#the scaling to pwm is done inside motor class
print('***Connect ESC Power')
print('***Wait beep-beep')

print('***then press ENTER')
res = raw_input()
mymotor.setW(0)
print('***Wait N beep for battery cell')
print('***Wait beeeeeep for ready')
print('***then press ENTER')
res = raw_input()
print ('increase > a | decrease > z | save Wh > n | set Wh > h|quit > 9 | set yourself > m')

cycling = True
try:
    while cycling:
        res = raw_input()
        if res == 'a':
            mymotor.increaseW()
        if res == 'z':
            mymotor.decreaseW()
        if res == 'n':
            mymotor.saveWh()
        if res == 'h':
            mymotor.setWh()
        if res == 'd':
            t = mymotor.__W
            for w in range(t, 1, -1):
                mymotor.__W = w
                mymotor.setW(w)
                time.sleep(0.5)
        if res == '9':
            cycling = False
        if res == 'm':
            w = int(raw_input())
            mymotor.__W = w
            mymotor.setW(w)


finally:
    # shut down cleanly
    mymotor.stop()
    print ("well done!")


