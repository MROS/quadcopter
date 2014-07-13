import sys
import os
import curses
from quadcopter import *

my_quadcopter = quadcopter()
my_quadcopter.init_hardware()

screen = curses.initscr()
curses.noecho()
screen.border(0)
screen.nodelay(1)

info_x = 110
info_y = 10

height, width = screen.getmaxyx()

def show_info(message):
    screen.addstr(height - 1, 0, message)
    screen.refresh()

def show_manual():
    manual1 = "w/a/s/d: control angle"
    manual2 = "< / >: add/sub master power by 1"
    manual3 = ", / .: add/sub master power by 3"
    start_point = (width - len(manual3)) / 2
    screen.addstr(height - 4, start_point, manual1)
    screen.addstr(height - 3, start_point, manual2)
    screen.addstr(height - 2, start_point, manual3)
def hide_manual():
    screen.addstr(height - 4, 1, " " * (width - 1))
    screen.addstr(height - 3, 1, " " * (width - 1))
    screen.addstr(height - 2, 1, " " * (width - 1))

def draw_info(info):
    screen.addstr(2, (width / 2) - 8, "main power: %d   " % my_quadcopter.motor_standard)
    screen.addstr(3, (width / 2) - 8, "pitch angle: %d   " % my_quadcopter.angle_standard[PITCH])
    screen.addstr(4, (width / 2) - 8, "roll angle: %d   " % my_quadcopter.angle_standard[ROLL])

    ten_x = 60; ten_y = 10
    screen.addstr(ten_y, ten_x + 10, "angle: %d   " % -my_quadcopter.angle_standard[PITCH])
    screen.addstr(ten_y + 12, ten_x + 10, "angle: %d   " % my_quadcopter.angle_standard[PITCH])
    screen.addstr(ten_y + 6, ten_x - 3, "angle: %d   " % -my_quadcopter.angle_standard[ROLL])
    screen.addstr(ten_y + 6, ten_x + 25, "angle: %d   " % my_quadcopter.angle_standard[ROLL])

    screen.addstr(ten_y + 1, ten_x + 10, "motor: %d   " % info.motors_speed['front'])
    screen.addstr(ten_y + 13, ten_x + 10, "motor: %d   " % info.motors_speed['rear'])
    screen.addstr(ten_y + 7, ten_x + -3, "motor: %d   " % info.motors_speed['left'])
    screen.addstr(ten_y + 7, ten_x + 25, "motor: %d   " % info.motors_speed['right'])

    gyro_x = 120; gyro_y = 15
    screen.addstr(gyro_y, gyro_x, "pitch rate: %f   " % info.rate[PITCH])
    screen.addstr(gyro_y + 1, gyro_x, "roll rate: %f   " % info.rate[ROLL])
    screen.addstr(gyro_y + 2, gyro_x, "yaw rate: %f   " % info.rate[YAW])
    

help = False

while True:
    key = screen.getch()
    if key == ord('q'):
        break
    elif key == ord('w'):
        my_quadcopter.angle_standard[PITCH] += 1
    elif key == ord('s'):
        my_quadcopter.angle_standard[PITCH] -= 1
    elif key == ord('a'):
        my_quadcopter.angle_standard[ROLL] += 1
    elif key == ord('d'):
        my_quadcopter.angle_standard[ROLL] -= 1
    elif key == ord(','):
        my_quadcopter.motor_standard -= 3
    elif key == ord('.'):
        my_quadcopter.motor_standard += 3
    elif key == ord('<'):
        my_quadcopter.motor_standard -= 1
    elif key == ord('>'):
        my_quadcopter.motor_standard += 1
    elif key == ord('h'):
        help = not help
        if help:
            show_manual()
        else:
            hide_manual()

    info = my_quadcopter.a_balance()
    draw_info(info)

    screen.refresh()

curses.nocbreak() 
screen.keypad(0)
caaurses.echo()
curses.endwin()
