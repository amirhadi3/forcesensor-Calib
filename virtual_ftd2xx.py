##### Python class to simulate the device (6-axis optical force/torque sensor) #######
## Use this when testing ftd2xx implementation
## Start this virtual device first, then the computer software meant to drive it
## To start virtual device, call `python virt_device.py` from a terminal instance
## Start the controlling script from a separate instance of terminal

import pylibftdi as ftdi
from numpy import random
import time
import sys
import CRC

INIT_BYTE = 'AA'
CRC4 = 0x03
BYTES_PER_DIGIT_IN = 2

class virtual_sensor:

	def __init__(self, portNum=2):

		#Now set up the real ports
		self.port = ftdi.Device('USB-COM485 Plus2', interface_select=portNum)
		self.port.ftdi_fn.ftdi_set_latency_timer(1)
		# self.port.setTimeouts(3,3)
		# self.port.setUSBParameters(64)
		self.port.baudrate =5000000
		self.reset()
		self.port.ftdi_fn.ftdi_usb_reset()
		print(self.port)

		self.num_errors = 0
		self.send_flag = False
		self.counter = 0
		self.counter2 = 0

		self.commands = {
			'12': self.sendPackage, #Data request
			'10': self.setSendFlag, #Start data transfer
			'11': self.setSendFlag, #Stop data transfer
			'E1': self.deac_transducer,
			'E2': self.deac_transducer,
			'E3': self.deac_transducer,
			'E4': self.deac_transducer,
			'E5': self.deac_transducer,
			'E6': self.deac_transducer,
			'F0': self.reset, #Full sensor reset
			'F1': self.reset_transducer,
			'F2': self.reset_transducer,
			'F3': self.reset_transducer,
			'F4': self.reset_transducer,
			'F5': self.reset_transducer,
			'F6': self.reset_transducer,
			'FA': self.reset_imu,
			'FB': self.reset_dac
			}

		self.crc4_table = CRC.calculate_CRC4_table()
		self.crc32_table = CRC.calculate_CRC32_table()

	def __del__(self):
		try:
			self.port.close()
		except AttributeError:
			print('Shutting down')

	def makeByte(self, value=-1, length=1):
		byte = ''
		if value == -1:
			for i in range(length):
				n = ''
				if i == 36: #Make a valid report ID
					n = random.randint(0,4)
					nums = ['01', '02', '04', '05', '08']
					n = nums[n]
				else:
					n = hex(random.randint(0,255))[2:]
					while len(n) < 2:
						n = '0' + n
				byte += n
		else:
			for i in range(length):
				n = hex(value)[2:]
				while len(n) < 2:
					n = '0' + n
				byte += n

		return byte

	def makePackage(self):
		"""
		Create random packet to simulate data returned by sensor
		Set first compnent of differential vector to a known number (65793)
		Set the initialization byte to 0xAA
		"""
		
		#Create random byte packet
		data = self.makeByte(length=47)

		#Calculate CRC-32 for the packet
		mainCRC = self.__calc_crc(data)

		if len(mainCRC) > 8:
			mainCRC = mainCRC[:8]
		while len(mainCRC) < 8:
			mainCRC = '0' + mainCRC

		#Generate CRC4 check for start bytes
		crc = self.__calc_crc( hex((int(INIT_BYTE,16) << 4) + self.counter)[2:], 4, CRC4)

		#  < Init byte > < 4-bit counter | CRC4 > < Byte 0 > ... < Byte 47 > < CRC32 >   
		msg = INIT_BYTE + hex(self.counter)[2:] + crc + data + mainCRC
		self.inc_counter()
		print(msg)
		
		return msg

	def sendPackage(self, command=None):
		"""
		Send the packet to the port
		"""
		self.port.write(self.makePackage())

	def setSendFlag(self, command):
		"""
		Set send flag to True if command was start_data_transfer
		Set to False if command was stop_data_transfer
		"""
		if command == '11':
			self.send_flag = False
			print('Stopping data transfer')
		else:
			self.send_flag = True
			print('Starting data transfer')

	def inc_counter(self):
		"""
		Simulate the 4 bit counter used for the initialization bytes
		"""
		self.counter += 1
		if self.counter == 16:
			self.counter = 0
			self.counter2 += 1
			if self.counter2 == 16:
				self.counter2 = 0
	
	def deac_transducer(self, command=None):
		"""
		Pretend to deactivate a transducer but don't actually do anything
		Only important thing here is that the correct transducer number is calculated
		"""
		trans_num = int(command, 16) - 224
		print('Transducer ' + str(trans_num) + ' deactivated')

	def reset(self, command=None):
		"""
		Reset input and output buffers
		"""
		self.port.ftdi_fn.ftdi_usb_reset()
		print('Sensor reset successfully')

	def reset_transducer(self, command):
		"""
		Pretend to reset a transducer but don't actually do anything
		Only important thing here is that the correct transducer number is calculated
		"""
		trans_num = int(command, 16) - 240
		print('Transducer ' + str(trans_num) + ' cleared')

	def reset_imu(self, command=None):
		"""
		Pretend to reset a IMU but don't actually do anything
		"""
		print('IMU reset successfully')

	def reset_dac(self, command=None):
		"""
		Pretend to reset a DAC but don't actually do anything
		"""
		print('DAC reset successfully')

	def config_trans(self, command=None):
		"""
		Pretend to configure a transducer but don't actually do anything
		"""
		print('Transfucer configured successfully')

	def config_imu(self, command=None):
		"""
		Pretend to configure an IMU but don't actually do anything
		"""
		print('IMU configured')

	def config_dac(self, command=None):
		"""
		Pretend to configure a DAC but don't actually do anything
		"""
		print('DAC configured')

	def pause_micro(self, us):
		for i in range(us*10):
			a=1

	def __calc_crc(self, p, n=32, polynomial=0x04C11DB7):
		"""
		Create n-bit CRC Checksum
		:param n the number of bits in the checksum (default is 32)
		:param p the bit sequence as a hex string, or int to create the checksum for 
			For example, bytes 0 to 46 in the 51 byte sensor packet
		:param polynomial the bit string of the CRC polynomial to use
			Default is the typical CRC-32 polynomial. CRC-4 often uses 0x03
		:return The n-bit checksum as a hexadecimal string
		"""
		p = self.toBytes(p)

		if n == 4:
			return "%x" % CRC.crc4(p,polynomial,self.crc4_table)
		elif n == 32:
			return "%x" % CRC.crc32(p,polynomial,self.crc32_table)

	def toBytes(self, string):
		"""
		Split a hexadecimal string of any length into a list of bytes
		"""

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

	def read(self, n):
		try:
			return self.port.read(n*BYTES_PER_DIGIT_IN)
		except ftdi._base.FtdiError:
			self.num_errors += 1
			if self.num_errors > 100:
				print('Error: connection failed. Shutting down...')
				self.port.close()
				sys.exit(0)
			print('Connection problem. Read failed')
			return ''

	def run(self):
		"""
		Simulate a sensor running in the background. Respond to commands when they come,
		otherwise wait and send data continuously if send_flag is True
		"""

		while (True):
			cmd = ''

			while(cmd == None or cmd == ''):
				#Wait for new command while executing current one (send packages if start_data_transfer was called)
				if self.send_flag:
					self.sendPackage()
					self.pause_micro(666) #Pause for 666us (approx 1500Hz)
				cmd = self.read(1)

			print(cmd)

			if str.startswith(cmd, '4'):
				self.config_dac()
				self.read(1)
			elif str.startswith(cmd, '3'):
				self.config_trans()
				self.read(1)
			elif str.startswith(cmd, '2'):
				self.config_imu()
				self.read(1)
			else:
				#Execute current command
				try:
					self.commands[cmd.upper()](cmd)
				except KeyError:
					print('Invalid Command')

			self.read(1)

		self.port.close() #close the port when done

#Start and run the sensor when this file is called ('python ftd2xx.py' at the command line)
s = virtual_sensor()
s.reset()
s.run()