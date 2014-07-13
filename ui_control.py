import curses
from quadcopter import *


my_quadcopter = quadcopter()
direction_map = {'a': 'left', 'd': 'right', 's': 'rear', 'w': 'front'}

screen = curses.initscr()

screen.border(0)
screen.nodelay(1)
screen.noecho()

height, width = screen.getmaxyx()
manual1 = "w/a/s/d: control angle"
manual2 = "< / > add/sub master power by 3"
manual3 = "< / > add/sub master power by 1"
screen.addstr(height - 3, manual1)
screen.addstr(height - 2, manual2)
screen.addstr(height - 1, manual3)
while True:
	a = screen.getch()
	info = my_quadcopter.a_balance()
	screen.refresh


curses.endwin()