#!/usr/bin/env python2
from quadcopter import quadcopter

my_quadcopter = quadcopter()
direction_map = {'a': 'left', 'd': 'right', 's': 'rear', 'w': 'front'}

def controller():
	while True:

		print("command: init | take_off | dsn | all #speed | (a|s|d|w) #speed | rollb")

		command = raw_input()
		if command == 'init':
			my_quadcopter.init_hardware()
		elif command == 'take_off':
			my_quadcopter.take_off()
		elif command == "rollb":
			my_quadcopter.roll_balance()
		elif command[0:3] == 'all':
			[co, speed] = command.split(' ')
			my_quadcopter.set_all_to(int(speed))
		elif command == 'dsn':
			my_quadcopter.descend()
		elif command[0] in direction_map:
			[mo_short, speed] = command.split(' ')
			mo = direction_map[command[0]]
			my_quadcopter.set_unique_to(mo, int(speed))
		else:
			print("wrong command!!!!\n")

controller()
