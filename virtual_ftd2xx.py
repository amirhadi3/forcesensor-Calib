##### Python class to simulate the device (6-axis optical force/torque sensor) #######
## Use this when testing ftd2xx implementation
## Start this virtual device first, then the computer software meant to drive it
## To start virtual device, call `python virt_device.py` from a terminal instance
## Start the controlling script from a separate instance of terminal

import pylibftdi as ftdi
from numpy import random
import time
import sys
import crc
import codecs
import struct

INIT_BYTE = 0xAA
BYTES_PER_DIGIT_IN = 1

class virtual_sensor:

	def __init__(self, portNum=2):

		#Now set up the real ports
		self.port = ftdi.Device('USB-COM485 Plus2', interface_select=portNum)
		self.port.ftdi_fn.ftdi_set_latency_timer(1)
		# self.port.setTimeouts(3,3)
		# self.port.setUSBParameters(64)
		self.port.baudrate = 5000000
		self.reset()
		self.port.ftdi_fn.ftdi_usb_reset()
		print(self.port)

		self.num_errors = 0
		self.send_flag = False
		self.counter = 0
		self.counter2 = 0

		self.commands = {
			0x12: self.sendPackage, #Data request
			0x10: self.setSendFlag, #Start data transfer
			0x11: self.setSendFlag, #Stop data transfer
			0xE1: self.deac_transducer,
			0xE2: self.deac_transducer,
			0xE3: self.deac_transducer,
			0xE4: self.deac_transducer,
			0xE5: self.deac_transducer,
			0xE6: self.deac_transducer,
			0xF0: self.reset, #Full sensor reset
			0xF1: self.reset_transducer,
			0xF2: self.reset_transducer,
			0xF3: self.reset_transducer,
			0xF4: self.reset_transducer,
			0xF5: self.reset_transducer,
			0xF6: self.reset_transducer,
			0xFA: self.reset_imu,
			0xFB: self.reset_dac
			}

		self.crc8_table = crc.calculate_CRC8_table()
		self.crc32_table = crc.calculate_CRC32_table()

	def __del__(self):
		try:
			self.port.close()
		except AttributeError:
			print('Shutting down')

	def makeByte(self, value=-1, length=1):
		byte = []
		if value == -1:
			for i in range(length):
				if i == 36: #Make a valid report ID
					n = random.randint(0,4)
					nums = [0x01, 0x02, 0x04, 0x05, 0x08]
					n = nums[n]
				else:
					n = random.randint(0,255)
				byte.append(n)
		else:
			for i in range(length):
				n = value
				byte.append(n)

		return byte

	def makePackage(self):
		"""
		Create random packet to simulate data returned by sensor
		Set first compnent of differential vector to a known number (65793)
		Set the initialization byte to 0xAA
		"""
		
		#Create random byte packet
		data = self.makeByte(length=49)

		#Calculate CRC-32 for the packet
		mainCRC = self.__calc_crc(data)

		#Generate CRC4 check for start bytes
		crc = self.__calc_crc([INIT_BYTE, self.counter], 8)

		#  < Init byte > < 8-bit counter > < CRC8 > < Byte 0 > ... < Byte 49 > < CRC32 >   
		msg = self.toBytes([INIT_BYTE, self.counter, crc] + data + mainCRC)
		self.inc_counter()
		
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
		if command == 0x11:
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

		if n == 8:
			return crc.crc8(p,polynomial,self.crc8_table)
		elif n == 32:
			csum = crc.crc32(p,polynomial,self.crc32_table)
			return [(csum & 0xFF000000)>>24, (csum & 0x00FF0000)>>16, (csum & 0x0000FF00)>>8, csum & 0x000000FF]

	def toBytes(self,byteList):
		"""
		The only reliable Python 2 and 3- compatible int-to-bytes conversion I could find 
		"""
		byte = ''
		if type(byteList) == int:
			byteList = [byteList]

		for num in byteList:
			byte += struct.pack("B", num)
		return byte

	def toBytesList(self, string):
		"""
		Split a hexadecimal string of any length into a list of bytes
		:param string the hexadecimal string to convert
		"""

		string = codecs.encode(string, 'hex')

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

	def read(self, n):
		try:
			return self.toBytesList(self.port.read(n*BYTES_PER_DIGIT_IN))
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
			cmd = []

			while(cmd == []):
				#Wait for new command while executing current one (send packages if start_data_transfer was called)
				if self.send_flag:
					self.sendPackage()
					self.pause_micro(666) #Pause for 666us (approx 1500Hz)
				cmd = self.read(4)

			print(cmd)
			cmd = cmd[0]

			#Execute current command
			try:
				self.commands[cmd](cmd)
			except KeyError:
				print('Invalid Command')

		self.port.close() #close the port when done

#Start and run the sensor when this file is called ('python ftd2xx.py' at the command line)
s = virtual_sensor()
s.reset()
s.run()
