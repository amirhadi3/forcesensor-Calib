import pylibftdi as ftdi
import time
import struct
import crc
import codecs
from sensor import Sensor
import sensor_output as out
import numpy as np
import matplotlib.pyplot as plt
import datetime

BAUD = 3000000
HZ = 1500
N_SAMPLES = 30000
PORT = 1
POLL = False
SAVE = True

def init():
	s = ftdi.Device(interface_select=PORT)
	s.baudrate=BAUD
	print(s.baudrate)
	# if e == -1:
	# 	print("Failed to set baudrate")
	s.ftdi_fn.ftdi_set_latency_timer(1)
	s.ftdi_fn.ftdi_set_line_property(8,1,0)
	return s

def test_poll(s):
	s.read(100) #Clear receive buffer

	start = time.time()
	s.write(b'\x12\x00\x00\x74')
	d = s.read(1)
	while d != b'\xaa':
		d = s.read(1) 
	c = s.read(2) #Ignore the crc4, counter
	d = s.read(53)
	print(time.time()-start)
	print(c)
	print(d)
	print(len(d))

def toBytesList(string):
		"""
		Split a hexadecimal string of any length into a list of bytes
		:param string the hexadecimal string to convert
		"""

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

def toHex(num):
		return "%x" % num

def read_in(s,n):
	data = s.read(n)
	while len(data) != n:
		data += s.read(n-len(data))
	data = codecs.encode(data,'hex') #Read n byte packet
	data = toBytesList(data)
	return data

def toStr(byte_list):
		string = b''
		for i in range(len(byte_list)):
					string += struct.pack("B", byte_list[i])
		string += struct.pack("B",crc.crc8(byte_list))
		return string

def to_int(byte):
	"""
	Helper method to convert a list of bytes where the least significant byte is 
	first or last into an int
	:param byte the list of bytes to convert to an int
	:param lsb_first True if the least significant byte of the number is listed first, False otherwise
		Default is false, so Most Significant Byte is assumed to be first
	"""
	num = 0
	sz = len(byte)

	for i in range(sz):
		num += (byte[sz - 1 - i] << i*8)

	return num

rep_ids = {
			0x1: 'Accelerometer',
			0x2: 'Gyroscope',
			0x4: 'Linear Acceleration',
			0x5: 'Rotation Vector',
			0x8: 'Game Rotation Vector'
			}
crc8_table = crc.calculate_CRC8_table()
crc32_table = crc.calculate_CRC32_table()

def test_cont(s):

	calMatrix = np.eye(6)
	count=0
	outer_timeout = N_SAMPLES * 10

	#Generate byte message with sample rate 
	hz = toHex(HZ)
	print(hz)
	while not len(hz) == 4:
		hz = '0' + hz
	b = b'' + toStr([0x10,int(hz[:2],16),int(hz[2:],16)])

	#Start timing, write, and read
	startt = time.time()
	s.write(b) 
	print('Started transmission')
	times=[]
	while count < N_SAMPLES and outer_timeout != 0:
		timeout = 100
		start = time.time()
		d = s.read(1)
		while d != b'\xAA' and timeout != 0:
			d = s.read(1)
			timeout -= 1
			# print('Read and ignored 1 byte: ' + str(d))
		
		if timeout == 0:
			outer_timeout -= 1
			continue

		d = codecs.encode(s.read(2), 'hex') #Read 2 bytes (counter and crc)
		# print(d)
		if len(d)==4 and int(d[2:],16) == crc.crc8(int('aa' + d[:2],16), table=crc8_table): #Check CRC 8
			data = read_in(s,53)
			if len(data)==53 and to_int(data[49:]) == crc.crc32(data[:49], table=crc32_table): #Check CRC 32
				try:
					rid = rep_ids.get(data[36])
				except KeyError:
					print('Invalid report ID key')

				out.sensor_output(None, data, rid, calMatrix)#.string()
				count += 1
			else:
				print('CRC-32 failed')
		else:
			print('CRC-8 failed')
		outer_timeout -= 1
		times.append(time.time()-start)

	s.write(b'\x11\x00\x00\xC9') #stop transmission
	count = 1
	while count < 100 and s.read(1) != '':
		s.write(b'\x11\x00\x00\xC9') #stop transmission
		count += 1
		s.ftdi_fn.ftdi_usb_purge_buffers()
	if count == 100:
		print('Failed to stop. Still sending data.')

	print(time.time()-startt)
	return times


###########################################################    Using Sensor Software    ################################################################################


def init2():
	return Sensor(calMatrix=np.eye(6),portNum=PORT)

def test_poll2(s):
	start = time.time()
	data = s.poll()
	#print(time.time()-start)
	if data != None:
		return time.time()-start
		#print('Packet read and parsed successfully')
	else:
		print('Failed to read packet')
	#print(data)

def test_cont2(s):
	start = time.time()
	s.start_data_transmission()
	i = 0
	timeout = 10000
	while i < N_SAMPLES and timeout > 0:
		timeout -= 1
		d = s.read()
		if d != None:
			i += 1
	print(time.time()-start)
	s.stop_data_transmission()

def now():
	n = datetime.datetime.now()
	return str(n.year) + '-' + str(n.month) + '-' + str(n.day) + '_' + str(n.hour) + '-' + str(n.minute) + '-' + str(n.second)

def do_stuff():
	times = []
	if POLL:
		s = init2()
		s.purge()
		if N_SAMPLES == 1:
			a = s.poll()
			a.string()
			return
		else:
			for i in range(N_SAMPLES):
				times.append(test_poll2(s))
	else:
		s = init2()
		# s.ftdi_fn.ftdi_usb_purge_buffers()
		# times = test_cont2(s)
		test_cont2(s)
		# s.write(b'\x11\x00\x00\xC9')

	# data = np.array(times)
	# plt.hist(data)
	# plt.show()

	if SAVE:
		if POLL:
			np.savetxt('poll_'+str(BAUD/1000000)+'mbps_'+str(N_SAMPLES)+'samples_' + now() + '.txt', data)
		else:
			np.savetxt('continuous_times_'+str(BAUD/1000000)+'mbps_'+str(HZ/1000)+'kHz_' + now() + '.txt', data) 

	count = 1
	while s.read(1) != '' and count < 1000:
		print('Clearing buffer')
		s.read(1000)
		count += 1

	if count == 1000:
		print('Failed. Sensor is still sending data')


if __name__ == '__main__':
	do_stuff()