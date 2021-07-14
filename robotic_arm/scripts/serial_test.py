#! /usr/bin/env python3

from serial import Serial
import time

BUFF_LEN = 255

SETUP = False

port = None

def write_ser(string):
	string = string + '\n'
	port.write(string.encode())

def read_ser(num_char = 1):
	read_ = port.read(num_char) 
	return read_.decode()

prev_time = time.time()
while(not SETUP):
	try:
		port = Serial("/dev/ttyUSB0", 115200, timeout = 1)
	except : #FileNotFoundError
		if((time.time() - prev_time) > 2):
			print("No serial port is open yet, plug in your uController!")
			prev_time = time.time()
	
	if(port is not None):
		SETUP = True


if(SETUP):
	done = False
	rep = ""
	while(not done):
		c = read_ser()
		rep += c
		if(not c):
			print(rep)
			done = True

while(True):
	read_data = read_ser(BUFF_LEN)

	if(len(read_data)):
		print(read_data)

	cmd = input()
	if(cmd):
		write_ser(cmd)
		print(f"Wrote: {cmd}")