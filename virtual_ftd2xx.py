##### Python class to simulate the device (6-axis optical force/torque sensor) #######
## Use this when testing ftd2xx implementation
## Start this virtual device first, then the computer software meant to drive it
## To start virtual device, call `python virt_device.py` from a terminal instance
## Start the controlling script from a separate instance of terminal

import ftd2xx
from numpy import random
import time
INIT_BYTE = 0xFF
CRC4 = 0x09

class virtual_sensor:

	def __init__(self, portNum=1):
		#I have no clue why this is necesary, but it is. Without this, F's are sometimes transmitted as 7s...?
		import pyftdi.serialext as serial
		tmpPortSensor = serial.serial_for_url('COM13', baudrate=5000000,timeout=1)
		tmpPortPC = serial.serial_for_url('COM12', baudrate=5000000,timeout=1)
		tmpPortSensor.close()
		tmpPortPC.close()

		#Now set up the real ports
		self.port = ftd2xx.open(portNum)
		self.port.setLatencyTimer(1)
		self.port.setTimeouts(3,3)
		self.port.setUSBParameters(64)
		self.port.setBaudRate(5000000)
		self.reset()
		self.port.resetPort()
		print(self.port)

		self.send_flag = False
		self.counter = 0
		self.counter2 = 0

		self.commands = {
			b'\x12': self.sendPackage, #Data request
			b'\x10': self.setSendFlag, #Start data transfer
			b'\x11': self.setSendFlag, #Stop data transfer
			b'\xF0': self.reset, #Full sensor reset
			b'\xF1': self.reset_transducer,
			b'\xF2': self.reset_transducer,
			b'\xF3': self.reset_transducer,
			b'\xF4': self.reset_transducer,
			b'\xF5': self.reset_transducer,
			b'\xF6': self.reset_transducer,
			b'\xFA': self.reset_imu,
			b'\xFB': self.reset_dac,
			}

	def __del__(self):
		self.port.close()

	def makePackage(self):
		"""
		Create random packet to simulate data returned by sensor
		Set first compnent of differential vector to a known number (65793)
		Set the initialization byte to 255 (0xff)
		"""
		#Create random byte packet
		data = bytes([self.counter2, self.counter, 0]) + bytes(random.randint(0,10,size=44).tolist())
		
		#Calculate CRC-32 for the packet
		mainCRC = self.__calc_crc(data)
		#Rearrange bytes to simulate sensor, which puts LSB first
		binc = bin(mainCRC)[2:]
		while len(binc) != 32:
			binc = '0' + binc
		mainCRC = bytes([int(binc[24:32],2)]) + bytes([int(binc[16:24],2)]) + bytes([int(binc[8:16],2)]) + bytes([int(binc[:8],2)])

		crc = self.__calc_crc( (INIT_BYTE << 4) + self.counter, 4, CRC4)
		#    < Init byte >    < 4-bit counter | CRC checksum >   < Byte 0 > ... < Byte 51 >   
		msg = bytes([INIT_BYTE]) + bytes([(self.counter << 4) + crc]) + data + mainCRC
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
		if command == b'\x11':
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

	def reset(self, command=None):
		"""
		Reset input and output buffers
		"""
		self.port.resetDevice()
		print('Sensor reset successfully')

	def reset_transducer(self, command):
		"""
		Pretend to reset a transducer but don't actually do anything
		Only important thing here is that the correct transducer number is calculated
		"""
		trans_num = int(command.hex(), 16) - 240
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

	def pause_micro(self, us):
		for i in range(us*10):
			a=1

	def __calc_crc(self, p, n=32, polynomial=0xedb88320):
		"""
		Create n-bit CRC Checksum
		:param n the number of bits in the checksum (default is 32)
		:param p the bit sequence in bytes, hex, or int to create the checksum for 
			For example, bytes 0 to 46 in the 51 byte sensor packet
		:param polynomial the bit string of the CRC polynomial to use
			Default is the typical CRC-32 polynomial. CRC-4 often uses 0x09
		:return The n-bit checksum
		"""

		#Convert to correct type and append an n bit buffer of 0s
		if type(p) == bytes:
			p = int(p.hex(),16) << n
		else:
		    p = p << n

		#Store the binary representation of p and the number of bits
		pBin = bin(p)[2:]
		length = len(pBin)

		#When p gets smaller than this, the dividend is zero (see Wikipedia)
		minVal = 2**n 

		#Shift the polynomial to align with the most significant bit
		poly = polynomial << (length - len(bin(polynomial)) + 2) #Plus 2 is for the extra 2 characters in the binary string: 0b...
		i = 0

		#Terminate when the dividend is equal to zero and only the checksum portion remains
		while p >= minVal: 
			#Shift the divisor until it is aligned with the most significant 1 in p (in binary)
		    while pBin[i] == '0' and i <= length - n: 
		        i = i + 1
		        poly = poly >> 1

		    #XOR the number with the divisor
		    p = p ^ poly 
		    #Update the bit string (used in the loop above)
		    pBin = bin(p)[2:]
		    #Make sure leading zeros weren't removed by Python
		    while len(pBin) < length:
		        pBin = '0' + pBin

		#Return the n-bit CRC checksum (last n bits of p)
		return int(pBin[n:],2) 

	def run(self):
		"""
		Simulate a sensor running in the background. Respond to commands when they come,
		otherwise wait and send data continuously if send_flag is True
		"""

		while (True):
			while(self.port.getStatus()[0] < 1):
				#Wait for new command while executing current one (send packages if start_data_transfer was called)
				if self.send_flag:
					self.sendPackage(self.port)
					self.pause_micro(100) #Pause for 100us
			
			cmd = self.port.read(1)
			print(cmd.hex()) #For debugging

			#Method of escaping the loop
			if cmd == b'\x66':
				break

			#Execute current command
			try:
				self.commands[cmd](cmd)
			except KeyError:
				print('Invalid Command')

		self.port.close() #close the port when done

#Start and run the sensor when this file is called ('python ftd2xx.py' at the command line)
s = virtual_sensor()
s.port.write(b'\x00')
s.reset()
s.run()