import pylibftdi as ftdi
import time
import struct
import sensor

def init():
	s = ftdi.Device(interface_select=0)
	s.baudrate=5000000
	s.ftdi_fn.ftdi_set_latency_timer(1)
	s.ftdi_fn.ftdi_set_line_property(8,1,0)
	return s

def test_poll(s):
	s.read(100) #Clear receive buffer

	start = time.time()
	s.write(b'\x12\x05')
	d = s.read(1)
	while d != 'aa':
		d = s.read(1) 
	s.read(1) #Ignore the crc4, counter
	d = s.read(51)
	print(time.time()-start)
	print(d)
	print(len(d))

def init2():
	return sensor.sensor(portNum=0)

def test_poll2(s):
	start = time.time()
	data = s.poll()
	print(time.time()-start)
	if data != None:
		print('Packet read and parsed successfully')
	else:
		print('Failed to read packet')
	print(data)

def test_cont(s):
	start = time.time()
	s.start_data_transmission()
	i = 0
	while i < 1500:
		d = s.read()
		if d != None:
			i += 1
	print(time.time()-start)