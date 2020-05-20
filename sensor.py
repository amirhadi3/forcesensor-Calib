import numpy as np
import sensor_output as out
import pylibftdi as ftdi
import crc
import threading
import time
import codecs
import struct
import os
import signal
from PyQt5 import QtCore
import re


INIT_BYTE = 0xAA
BYTES_PER_DIGIT_IN = 1
"""
Implementation of sensor class using ftd2xx instead of pyftdi.serialext.
This is probably the best because the latency timer can be set programmatically
To use, initialzie a sensor object, send commands and receive sensor output by calling
the functions below
"""

data_stream = None
class Sensor:
	
	def __init__(self, calMatrix, portNum=2, quick=False):
		"""
		Opens a connection to a serial device and creates a serial object with which to control it
		:param portNum Index of port of device to open. Default value is 1, but 2 is also common
		"""

		#Dictionary of report IDs
		self.rep_ids = {
			1: 'Accelerometer',
			2: 'Gyroscope',
			4: 'Linear Acceleration',
			5: 'Rotation Vector',
			8: 'Game Rotation Vector'
			}

		self.ab = {
			2.5:(0x5DC000,2.7304),
			5:(0x5DC000,2.7304),
			10:(0x5DC000,2.7304),
			15:(0x3E8000,1.8202),
			25:(0x4B0000,2.1843),
			30:(0x3E8000,1.8202),
			50:(0x4B0000,2.1843),
			60:(0x3E8000,1.8202),
			100:(0x4B0000,2.1843),
			500:(0x3C0000,1.7474),
			1000:(0x3C0000,1.7474),
			2000:(0x3C0000,1.7474),
			3750:(0x400000,1.8639),
			7500:(0x400000,1.8639),
			15000:(0x400000,1.8639),
			30000:(0x400000,1.8639)
		}

		self.crc8_table = crc.calculate_CRC8_table()
		self.crc32_table = crc.calculate_CRC32_table()
		self.calMatrix = calMatrix

		self.num_errors = 0
		self.port = ftdi.Device('USB-COM485 Plus2', interface_select=portNum)
		self.port.ftdi_fn.ftdi_set_latency_timer(1)
		self.port.ftdi_fn.ftdi_set_line_property(8,1,0)
		self.port.baudrate = 3000000

		self.threadLock = threading.Lock()
		self.thread = read_thread(parent=self, port=self.port, threadLock=self.threadLock)
		self.block = False

		self.inBuf = [False]*6
		self.data_rate = 1500
		self.adsGain = [1]*6
		self.adsRate = [30e3]*6
		self.vref = 2.5
		self.OFC = [1]*6
		self.FSC = [1]*6
		if not quick:
			for i in range(6):
				o,f = self.ads_report_registers(i)
				self.OFC[i] = o
				self.FSC[i] = f

	#Close session
	def __del__(self):
		try:
			self.port.close()
		except AttributeError:
			print('Shutting down')
		print('Connection closed')
	
	def disconnect(self):
		self.port.close()

	def read(self, timeout=10000):
		"""
		Read and parse 53 bytes from the continuous data stream.
		This should be called in a loop after calling start_data_transmission()
		:return sensor_output object of the data
			returns Empty sensor_output if it was unable to read 53 bytes from the stream
		"""
		global data_stream

		#Wait until timeout for data
		count = 0
		while data_stream == None and count < timeout:
			count += 1

		if count == timeout:
			print('Error: Error reading from stream (No data)')
			return out.sensor_output()
		else:
			self.threadLock.acquire()
			data = data_stream
			data_stream = None #Get rid of the old measurement
			self.threadLock.release()
		return data

	def write(self, byte):
		try:
			n = self.port.write(byte)
			print('Sent ' + str(n) + ' bytes (' + codecs.encode(byte,'hex') + ')')
		except ftdi._base.FtdiError:
			print("Failed to write. USB bulk write error.")

	####### Functions of the sensor  #######
	###  Measurements  ###
	def start_data_transmission(self, data_rate=1500):
		"""
		Start continuous transmission of data from the sensor by starting a thread
		that reads continuously without blocking and writes to a queue
		"""
		if self.block:
			print('Failed. In continuous data mode already. Call stop_data_transmission() first.')
			return
		data_rate = self.toHex(data_rate)
		while len(data_rate) < 4:
			data_rate = '0' + data_rate

		byte1 = int(data_rate[:2],16)
		byte2 = int(data_rate[2:],16)

		#Prompt the start of continuous data transmission
		print('Starting data transmission at ' + str(int(data_rate,16)) + 'Hz')
		self.port.write(self.toStr([0x10, byte1, byte2], with_crc8=True))
		self.block = True
		self.thread.start()

	def stop_data_transmission(self):
		"""
		Stop continuous transmission of data from the sensor by ending the thread
		and clearing all associated buffers to avoid receiving extra data after the fact.
		The data stream remains untouched.
		"""

		# Tell device to stop
		self.block = False
		
		self.thread.exitFlag = True #End the thread
		# if self.thread.is_alive():	
		# 	self.thread.join()
		if self.thread.isRunning():
			self.thread.quit()
			time.sleep(0.5)
			if self.thread.isRunning():
				print("Error. Read thread still running")

		self.__do([0x11, 0x00, 0x00],regex='\w+-\w+-[SF]',verbose=False)
		count = 1
		while count < 100 and self.readBytes(1) != []:
			self.__do([0x11,0x00,0x00], verbose=False,regex='\w+-\w+-[SF]')
			self.purge_rx()
			count += 1
		if count == 100:
			print('Failed to stop. Still sending data.')

		self.purge(verbose=True)

		#Destroy the thread and create a new one so it can be started again
		del self.thread
		self.thread = read_thread(parent=self, port=self.port, threadLock=self.threadLock)

	def poll(self):
		#Take one measurement and parse it
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		dat = self.__do([0x12, 0x00, 0x00, 0x74],verbose=False,expect_package=True,with_crc8=False,fast=True)
		if dat != None:
			# dat.string(detailed=True)
			return dat
		else:
			print('Failed to read data. No package received.')
	
	###  Reset and Deactivation  ###

	def reset_device(self):
		#Reset the device
		return self.__do([0xF0, 0x00, 0x00],regex='\w+-\w+-[SF]')
	
	def reset_imu(self):
		#Reset the IMU
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		return self.__do([0xFA, 0x00, 0x00],regex='\w+-\w+-[SF]')
	
	def reset_dac(self):
		#Reset the DAC
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		return self.__do([0xFB, 0x00, 0x00],regex='\w+-\w+-[SF]')
	
	def reset_ads(self, ads_num):
		"""
		Reset and activate one of the ADS1257 adss
		:param ads_num the index of the ads to reset (0-5)
		"""
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		if ads_num <= 5 and ads_num >= 0: 
			resp = self.__do([0xF0 + ads_num + 1, 0x00, 0x00],regex='\w+-\w+-[SF]')
			time.sleep(0.7)
			o,f = self.ads_report_registers(ads_num)
			if 2**17 == o or 2**17 == f:
				print('Updating calibration constants failed. Please Retry')
			else:
				self.OFC[ads_num] = o
				self.FSC[ads_num] = f
			return resp + 'ADS' + str(ads_num+1) + ' recalibrated successfully'
		elif ads_num == -1:
			resp = ''
			for i in range(6):
				resp+=self.reset_ads(i)
			return resp
		else: 
			print('Invalid index')
	
	def deactivate_ads(self, ads_num):
		"""
		Deactivate one of the ADS1257 ads's
		:param ads_num the index of the ads to deactivate (0-5)
		"""
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		if ads_num == -1:
			resp = ''
			for i in range(6):
				resp += self.__do([0xE0 + i+1, 0x00, 0x00],regex='\w+-\w+-[SF]')
			return resp
		if ads_num < 5 and ads_num >= 0: 
			return self.__do([0xE0 + ads_num+1, 0x00, 0x00],regex='\w+-\w+-[SF]')
		else: 
			print('Invalid index')

	###  Configure Parts  ###
	def config_imu(self, mode, delay):
		"""
		Configure the IMU output
		:param mode the type of output to set the IMU to output
			Mode can be given as the string or int from the list below:
				1	Accelerometer
				2	Gyroscope
				4	Linear Acceleration
				5	Rotation Vector
				8	Game Rotation Vector
			The strings are not case sensitive and only the first 3 letters are considered
			so 'acc', 'accel', 'Acc', 'acceleration', 'AcCeletometer' are all valid strings for mode 1
		:param delay the interval between reports in ms (up to 16 bits)
		"""
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		byte = []
		interv = self.toHex(delay)
		while len(interv) < 4:
			interv = '0' + interv

		delay1 = int(interv[:2], 16)
		delay2 = int(interv[2:], 16)

		if type(mode) == unicode or type(mode) == str:
			mode = mode.lower()
			if mode.startswith('acc'):
				msg = 0x21
			elif mode.startswith('gyr'):
				msg = 0x22
			elif mode.startswith('lin'):
				msg = 0x24
			elif mode.startswith('rot'):
				msg = 0x25
			elif mode.startswith('gam'):
				msg = 0x28
			else:
				print('Invalid IMU Configuration Mode')
				return
			byte = [msg, delay1, delay2]
		elif type(mode) == int:
			byte = [0x20 + mode ,delay1, delay2]
		else:
			return
		return self.__do(byte,regex='\w+-\w+-[SF]')

	def set_imu_accelerometer(self, delay):
		return self.config_imu(1, delay)

	def set_imu_gyroscope(self, delay):
		return self.config_imu(2, delay)

	def set_imu_lin_accel(self, delay):
		return self.config_imu(4, delay)

	def set_imu_rotation(self, delay):
		return self.config_imu(5, delay)

	def set_imu_game_rot(self, delay):
		return self.config_iu(8, delay)

	def imu_start_calibration(self):
		self.__do([0x2F, 0x01, 0x00],regex='\w+-\w+-[SF]')

	def imu_cancel_calibration(self):
		"""
		Stop the calibration without saving it
		"""
		self.__do([0x2F, 0x03, 0x00],regex='\w+-\w+-[SF]')

	def imu_finish_calibration(self):
		"""
		Complete the IMU calibration, saving the Dynamic Calibration Data (DCD0 to the IMU)
		"""
		self.__do([0x2F, 0x02, 0x00],regex='\w+-\w+-[SF]')

	def parse_calibration_state(self):
		"""
		Parse the responses given by the sensor during calibration
		:return Status, statusType where status is the accuracy of the game rotation vector/magnetic field output
			and statusType is which mode is being reported (mag or grvec)
			Accuracy is given as:
				0		Unreliable
				1		Accuracy Low
				2		Accuracy Medium
				3		Accuracy High 
		"""
		dat = self.readLine(startChar='G', startChar2='M')

		datStr = ''
		for num in dat:
			datStr += self.toHex(num, padded=True)

		datStr = datStr.decode('hex')

		idx = datStr.find(':')

		try:
			status = dat[idx+1]
			return status-48, datStr[:idx]	
		except:
			return -1, ''


	def __config_ads(self, ads_num, category, setting):
		"""
		Configure the ADS1257-x device(s). These are adss with built in PGAs.
		:param ads_num specifies which of the six adss to control
			Indexing is 0-5. -1 targets every ads at once
		:param category what configuration category to change. 
			The following are the available categories:
				0	'root' 		configure the root registers
				1	'drate'		configure the data rate
				2	'pga'		configure the PGA gain
				3	'pos'		configure the positive channel
				4	'neg'		configure the negative channel
			These names are not case sensitive, and the category can be selected 
			either by passing the string or the number from the above list 
		:param setting what to change the selected category's setting to
			The following are the available settings, organized by their categories
				0-Root Register Configuration:
					This setting should be passed as a binary tuple or string. The first element or character
					enables (1) or disables (0) ACAL. The second enables (1) or disables (0) IN-BUFF. 
						For example, (1,0) or '10' would both enable ACAL and disable IN_BUFF
				1-Data Rate Configuration
					The following rates in SPS are permitted. Any other number will be 
					rounded to the nearest value from this list:
						2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1e3, 2e3, 3.75e3, 7.5e3, 15e3, 30e3
				2-PGA Gain Configuration
					The following PGA Gains are permitted. Any other number will be 
					rounded to the nearest value from this list:
						1, 2, 4, 8, 16, 32, 64
				3-Positive Channel Configuration
					Enter the int x to select AINx where x is 0, 1, 2, or 3
				4-Negative Channel Configuration
					Enter the int x to select AINx where x is 0, 1, 2, or 3
				6- Issues self-calibration 
		:param sensor_num specifies which sensor this is done to. -1 indicates all sensors
		""" 
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return
		if ads_num not in [-1,0,1,2,3,4,5]:
			print('Invalid ads number. Values between 1 and 6 are permitted, or -1 to select all')
			return
		if ads_num == -1:
			resp = ''
			for i in range(6):
				resp += self.__config_ads(i,category,setting)
			return resp
		else:
			#Set the first byte
			byte = [0x30 + ads_num+1] 
			resp = []
			#Select which category to work with
			cat_dict = {'root' : 0, 'drate' : 1, 'pga' : 2, 'pos' : 3, 'neg' : 4}
			if type(category) == str: #Convert string inputs
				category = cat_dict.get(category.lower(), value=-1)
			if category < 0 or category > 6: #Check validity
				print('Invalid Category String')
				return

			if category == 0: #Configure Root Registers
				if (type(setting) != tuple and type(setting) != str) or len(setting) != 2 or setting[0] not in [0,1] or setting[1] not in [0,1]:
					print('Invalid setting. Please enter a tuple or string with 2 binary elements to configure root registers')
					return
				self.inBuf[ads_num] = setting[1]==1
				config = int(str(setting[0]) + str(setting[1]) + '000000',2)
				byte.append(0x00)
				byte.append(config)

			elif category == 1: #Configure Data Rate
				rates = np.array([2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1e3, 2e3, 3.75e3, 7.5e3, 15e3, 30e3])
				#Ensure we have an acceptable value
				if setting not in rates:
					diff = np.abs(rates - setting)
					setting = rates[np.argmin(diff)]
					print('Data rate value not permitted. Rounding to ' + str(setting))

				self.adsRate[ads_num] = setting

				config = np.where(rates == setting)[0][0]
				byte.append(0x01)
				byte.append(config)

			elif category == 2: #Configure PGA Gain
				gains = np.array([1, 2, 4, 8, 16, 32, 64])
				
				if setting not in gains:
					diff = np.abs(gains - setting)
					setting = gains[np.argmin(diff)]
					print('Gain value not permitted. Rounding to ' + str(setting))

				self.adsGain[ads_num] = setting

				config = np.where(gains == setting)[0][0]
				byte.append(0x02)
				byte.append(config)

			elif category == 3: #Configure Positive Channel
				if setting not in [0,1,2,3]:
					print('Please enter a channel between 0 and 3 (inclusive)')
					return
				byte.append(0x03)
				byte.append(setting)

			elif category == 4: #Configure Negative Channel
				if setting not in [0,1,2,3]:
					print('Please enter a channel between 0 and 3 (inclusive)')
					return
				byte.append(0x04)
				byte.append(setting)

			elif category == 6: #self-calibrate
				byte.append(0x06)
				byte.append(0x00)

			resp = self.__do(byte,regex='\w+-\w+-[SF]')

			o,f = self.ads_report_registers(ads_num)
			if o==2**25 or f==2**25:
				print('Updating calibration constants failed. Please Retry')
			else:
				self.OFC[ads_num] = o
				self.FSC[ads_num] = f

			return resp

	def set_ads_drate(self, ads_num, data_rate):
		"""
		Convenience method that calls config_ads to set the data rate
		:param ads_num which ads to do this to (1-6)
		:param data_rate the desired data rate. If this is not one allowed by config_ads,
			it will be rounded to the nearest allowed value
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		return self.__config_ads(ads_num, 1, data_rate)
		# self.ads_self_calibrate(ads_num)

	def set_ads_registers(self, ads_num, ACAL, IN_BUFF):
			"""
			Convenience method that calls config_ads to configure the root registers
			:param ads_num which ads to do this to (1-6)
			:param ACAL 1 to enable ACAL, 0 to disable it
			:param IN_BUFF 1 to enable IN_BUFF, 0 to disable it
			:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
			"""
			return self.__config_ads(ads_num, 0, (ACAL, IN_BUFF))
			# self.ads_self_calibrate(ads_num)

	def set_pga_gain(self, pga_num, gain):
		"""
		Convenience method that calls config_ads to set the PGA's gain
		:param pga_num which PGA to do this to (1-6)
		:param gain the desired gain value. If this is not one allowed by config_ads,
			it will be rounded to the nearest allowed value
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		return self.__config_ads(pga_num, 2, gain)
		# self.ads_self_calibrate(pga_num)

	def set_ads_channel(self, ads_num, channel, positive=True):
		"""
		Convenience method that calls config_ads to set the positive or negative channel number
		:param ads_num which ads to do this to (1-6)
		:param channel the desired channel number (0-3)
		:param positive flag that states whether to change the positive or negative channel
			True changes the positive channel, false does the negative one
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		if positive:
			return self.__config_ads(ads_num, 3, channel)
		else:
			return self.__config_ads(ads_num, 4, channel)

	def ads_self_calibrate(self, ads_num):
		"""
		Convenience method that calls config_ads to self-calibrate the selected ADS
		:param ads_num which ads to do this to (1-6)
		"""
		return self.__config_ads(ads_num, 6, 0)

	def ads_report_registers(self, ads_num):
		"""
		Reads the calibration registers of the ADS and reports the OFC and FSC registers 
		:param ads_num which ads to do this to (1-6)
		:return OFC register value, FSC register value
			-1 for both if reading the response failed
		"""
		
		if ads_num == -1:
			for i in range(6):
				ofc, fsc = self.ads_report_registers(i)
			return ofc, fsc

		resp = self.__do([0x30+ads_num+1,0x05,0x00], toHex=False, regex='\w+-\w+-[SF]')
		ok = False
		startIndex = 0
		try:
			startIndex = resp.index(0xAA)+1
			ok = self.__check_crc(resp[startIndex+6],resp[startIndex-1:startIndex+6],8)
		except ValueError:
			print('Failed to report FSC and OSC registers')
			return 2**25, 2**25

		if ok:
			ofc = self.toInt(resp[startIndex:startIndex+3],lsb_first=True)
			#Check if ofc is negative and find 2's complement accordingly
			if ofc >= 2**23:
				ofc -= 2**24
			fsc = self.toInt(resp[startIndex+3:startIndex+6],lsb_first=True)
			return ofc, fsc
		else:
			print('Failed to report FSC and OSC registers: Checksum failed')
			return 2**25, 2**25

	def config_dac(self, channel, voltage):
		"""
		Configure the DAC by setting the output voltage of a specified channel or powering off a channel
		:param channel the channel to set the voltage of (int between 1 and 6, inclusive)
		:param voltage the voltage to set the output of the selected channel to. This should be between 0 and 50mA
			input 0 to shut a channel off
		"""
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return

		if channel == -1:
			resp = ''
			for i in range(6):
				resp += self.config_dac(i, voltage)
			return resp
		else:
			voltage = voltage * 0xFFF / 50

			config = []

			if channel < 0 or channel > 5:
				print('Invalid channel number. Input a number between 1 and 7, inclusive')
				return

			channel += 1
			if voltage == 0:
				config = [0x47, channel<<4, 0x00]
			else:
				volts_hex = self.toHex(voltage)

				#Adjust the length to 12 bits
				while len(volts_hex) < 3:
					volts_hex = '0' + volts_hex

				config = [(4<<4) + channel, int(volts_hex[:1],16), int(volts_hex[1:], 16)]
			return self.__do(config, with_crc8=True, regex='\w+-\w+-[SF]',fast=True)

	def turn_on_led(self, ledNum):
		return self.config_dac(ledNum, 20)
	def turn_on_leds(self,ledNums):
		resp = ''
		for num in ledNums:
			resp += self.config_dac(num, 20)
		return resp
	def turn_on_leds_all(self):
		resp = ''
		for i in range(6):
			resp += self.config_dac(i,20)
		return resp
	def turn_off_led(self, ledNum):
		return self.config_dac(ledNum, 0)
	def turn_off_leds(self,ledNums):
		resp = ''
		for num in ledNums:
			resp += self.config_dac(num, 0)
		return resp
	def turn_off_leds_all(self):
		resp = ''
		for i in range(6):
			resp += self.config_dac(i,0)
		return resp

	#########   Data transmission #######

	def __do(self, task, verbose=True, expect_package=False, with_crc8=True, toHex=True, regex=None, fast=False):
		"""
		Send an arbitrary command given by task and potentially read the response. This method is thread safe and acquires/releases a lock around its read/write operations so the serial communication does not get messed up 
			:param task byte to send to device 
			:param verbose indicates whether or not to print the bytes received before finding an init byte or timing out 
			:param expect_package indicates whether to attempt to read in and parse a 53 byte packet after sending the command 
			:param with_crc8 indicates whether to append a crc8 to the message (True), or not (False) 
			:return parsed response from sensor if expect_package is True. None otherwise 
		"""
		#Write the command, ensuring it is a bytes object, and that no other thread is trying to write at this time
		self.threadLock.acquire()
		toWrite = self.toStr(task, with_crc8=with_crc8)
		self.write(toWrite)

		if not fast:
			time.sleep(0.2)
		else:
			time.sleep(0.001)

		if expect_package:
			timeout = 200
			flag = self.wait_for_packet(timeout, verbose=verbose)
			if flag:
				data = self.readBytes(53)
				self.threadLock.release()
				return self.__parse(data) if (data != None and len(data) == 53) else None
			else: #If it did time out, return empty sensor output
				self.threadLock.release()
				return None
		else:
			resp = self.read_packet(timeout=300)
			h = ''
			if resp != []:
				for num in resp:
					h += self.toHex(num, padded=True)
				h = h.decode('hex')

				#Try to match regex
				if regex != None:
					match = re.findall(regex,h)
					if match != []:
						h = ''
						count = 0
						for string in match:
							h += string + '. '
							count += 1
							if count % 4 == 0:
								h += '\n'
						if h[-1] != '\n':
							h += '\n'

			if verbose:
				print(h)
			self.threadLock.release()
			if toHex:
				return h
			else:
				return resp

	def wait_for_packet(self, timeout=50, verbose=False):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 53 bytes as a packet.
		:param timeout how many bytes to attempt before stopping and giving up, returning False
		:param verbose Set to true to print out all the extra bytes that were read in before receiving a star byte or timing out
		:return True if the program can read in the next 53 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		data = ''
		for i in range(timeout):
			#Check for initialization byte
			dat = self.readBytes(1) 
			if dat == [INIT_BYTE]:
				#Print all the garbage that was read in
				if verbose and data != '':
					print(data)

				#Read next 2 bytes (counter and crc8)
				byte = self.readBytes(2)
				if byte != None and byte != []:
					counter = byte[0]
					crc = byte[1]
				else:
					continue

				#Combine initialization byte and counter
				p = [INIT_BYTE, counter]

				#Test checksum
				return self.__check_crc(crc,p,8)
			elif dat == [0]:
				i -= 1
			else:
				if dat != [] and dat != None:
					data += self.toHex(dat[0],padded=True)

		#Print all the garbage that was read in
		if verbose and data != '':
			print(data.decode('hex'))
		return False

	def read_packet(self, timeout=200):
		"""
		Read arbitrary package and return as list of integer bytes
		"""
		count = 0
		d = self.readBytes(1)
		data = []
		#Wait for start of package
		while d == []:
			d = self.readBytes(1)
			count = count + 1
			if count == timeout:
				print('No response received')
				return data

		count = 0
		failed = 0
		while failed < 20:
			if d == []:
				failed += 1
			else:
				data.append(d[0])
				count = count + 1
				if count > timeout:
					break
			d = self.readBytes(1)

		return data

	###########    Helper Methods    ###########

	def purge_rx(self,verbose=True):
		if self.port.ftdi_fn.ftdi_usb_purge_rx_buffer() == 0:
			if verbose:
				print("Receive buffer cleared")
			return 0
		else:
			print("Error clearing receive buffer")
			return -1
	def purge_tx(self, verbose = True):
		if self.port.ftdi_fn.ftdi_usb_purge_tx_buffer() == 0:
			if verbose:
				print("Transmit buffer cleared")
			return 0
		else:
			print("Error clearing transmit buffer")
			return -1

	def purge(self, verbose=True):
		return self.purge_tx(verbose) + self.purge_rx(verbose)

	def __check_crc(self, crc_, p, n=32):
	    """
	    Check crc Checksum with 4 or 32 bits
	    :param crc the n bit checksum as a list of bytes (ints) or an int
	    :param p the list of bytes to compare to the checksum. (This is bytes 0 to 46 in the 53 byte sensor packet)
	    :param polynomial the bit string of the crc polynomial to use
	    	Default is 0x04C11DB7 which is what we use for n=32. For n=4, 0x03 is commonly used
	    :return True if the checksum matches, False otherwise
	    """

	    if type(p) == int:
	    	p = self.toBytesList(self.toHex(p))
	    if type(crc_) != int:
    		crc_ = self.toInt(crc_)

	    if n == 8:
	    	checksum = crc.crc8(p, table=self.crc8_table)
	    elif n == 32:
	    	checksum = crc.crc32(p, table=self.crc32_table)
	    return checksum == crc_

	def __reset(self,timeout=10):
		"""
		Reset all input and output buffers until there are no more bytes waiting.
		If the buffer keeps filling up with new data after the reset, this will try
		100 times to clear the buffers before it gives up because something is continuosly
		writing to the buffer
		"""
		i = 0
		while len(self.readBytes(1)) != 0 and i < 100:
			self.port.ftdi_fn.ftdi_usb_reset()
			i += 1

		if i == timeout:
			print('Failed to reset. Ensure nothing is actively sending data')


	def __parse(self, packet):
		"""
		Parse the data packet of 53 bytes using the SH-2 structure
		:return sensor_output object which contains the parsed data in a useful form
			returns a sensor_output object containing only the original received packet
				if the packet length is not 53 bytes
			returns None if the crc checksum did not match
		"""

		if not self.__check_crc(packet[49:],packet[:49]):
			print('Checksum failed')
			return out.sensor_output()
		
		rid = 'None'

		try:
			rid = self.rep_ids.get(packet[36])
		except KeyError:
			print('Invalid report ID key')

		return out.sensor_output(self, packet, rid, self.calMatrix)

	def readBytes(self, num_bytes):
		if self.block:
			print('Failed. In continuous data mode. Call stop_data_transmission() first.')
			return

		try:
			read_in = self.port.read(num_bytes * BYTES_PER_DIGIT_IN)
		except ftdi._base.FtdiError:
			print('FTDI Error thrown while reading')
			self.num_errors += 1
			if self.num_errors == 100:
				raise ftdi._base.FtdiError('Failed too many times. Exiting...')
			return []
		try:
			retval = self.toBytesList(read_in)
			return retval
		except ValueError:
			return []

	def readLine(self, startChar=None, startChar2=None, endChar='\n', timeout=100):
		"""
		Attempt to read one line from serial, starting with startChar, and ending with endChar
		:param startChar the character to expect as the first character of the line.
			None by default allows any character to be the first character
		:param endChar the character at which to stop reading
			By default the new line character, '\n'
		:param timeout how many bytes to read, looking for the startChar and then endChar before giving up
		"""
		if startChar != None:
			startByte = int(codecs.encode(startChar, 'hex'),16)
			startByte2 = int(codecs.encode(startChar2, 'hex'),16)
		if endChar != None:
			endByte = int(codecs.encode(endChar, 'hex'),16)

		#Look for the start byte
		d = self.readBytes(1)
		count = 0
		while d != [startByte] and d != [startByte2] and count < timeout:
			d = self.readBytes(1)
			count += 1

		#Exit if it couldn't be found
		if count == 1000:
			return []

		#Look for the end byte
		line = []
		count = 0
		while d != [endByte] and count < timeout:
			if d != []:
				line.append(d[0])
			d = self.readBytes(1)
			count += 1

		#Return what came between start and end (inclusive)
		return line

	def toInt(self, byte, lsb_first=False):
		"""
		Helper method to convert a list of bytes where the least significant byte is 
		first or last into an int
		:param byte the list of bytes to convert to an int
		:param lsb_first True if the least significant byte of the number is listed first, False otherwise
			Default is false, so Most Significant Byte is assumed to be first
		"""
		num = 0
		sz = len(byte)

		if lsb_first: #LSB is first
			for i in range(sz):
				num += (byte[i] << i*8)
		else: #LSB is last
			for i in range(sz):
				num += (byte[sz - 1 - i] << i*8)

		return num

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

	def toStr(self, byte_list, with_crc8=False, format=0):
		"""
		Do the opposite of toBytesList. Convert a list of bytes (array of 8-bit ints in decimal)
		to a byte string in hex.
		:param byte_list the list of bytes (as decimal ints) to convert to a byte string
		:param with_crc8 True if a crc8 checksum for the number given by byte_list should be appended to the string
			Default is False, so no crc8 is computed or added.
		:param format can take the value 0 (bytes) or 1 (string))
			If format = 0, numbers will undergo the following conversion: 15 -> 0x0f -> b'\x0f' which is a bytes
			object useful for sending to the sensor. This works in Python 2.7 and 3
			If format = 1, numbers will be converted directly to a hex string: 15 -> '0f', which is actually 2 bytes b'\x30\x66'.
			These are the ASCII values of the characters. This is not useable for sending to a sensor.
		"""
		string = b''
		if format == 0:
			string = self.toBytes(byte_list)
		else:
			for i in range(len(byte_list)):
					string += self.toHex(byte_list[i])

		if with_crc8 == True:
			if format == 0:
				string += self.toBytes(crc.crc8(byte_list, table=self.crc8_table))
			else:
				string += self.toHex(crc.crc8(byte_list, table=self.crc8_table))

		return string

	def toHex(self, num, padded=False):
		"""
		Convert to hex without the "0x" at the start or the random "L" at the end
		"""
		h = "%x" % num
		if padded:
			if len(h) % 2 != 0:
				h = h + '0'
			return h
		else:
			return h

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

################################################################################################################
################################################################################################################
################################################################################################################

# class read_thread (threading.Thread):
class read_thread (QtCore.QThread):
	"""
	Class that inherits threading and defines the behaviour of a separate thread to read data
	continuously without blocking the main thread. To start this thread, first initiate it
	using the constructor thread = read_thread(port, lock). Then use thread.start(), which executes
	the run function in a new thread
	"""
	data_source = QtCore.pyqtSignal(object)

	def __init__(self, parent, port, threadLock, threadID=0, name='read_thread'):
		QtCore.QThread.__init__(self)#threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.port = port
		self.error_count = 0
		self.threadLock = threadLock
		self.exitFlag = False
		self.numAdded = 0
		self.parent = parent
		self.rep_ids = {
			1: 'Accelerometer',
			2: 'Gyroscope',
			4: 'Linear Acceleration',
			5: 'Rotation Vector',
			8: 'Game Rotation Vector'
			}

	def readBytes(self, num_bytes):
		try:
			read_in = self.port.read(num_bytes)
			timeout = 100
			while len(read_in) != num_bytes and timeout > 0:
				read_in += self.port.read((num_bytes-len(read_in)))
				timeout -= 1
			if timeout == 0:
				raise ftdi._base.FtdiError
		except ftdi._base.FtdiError:
			self.error_count += 1
			if self.error_count == 10:
				raise ftdi._base.FtdiError('Failed too many times. Exiting...')
			return None
		try:
			retval = self.toBytesList(read_in)
			return retval
		except ValueError:
			print('Error converting received data to bytes')
			return None

	def toHex(self, num, padded=False):
		if num == b'':
			return ''
		h = "%x" % int(num)
		if padded:
			if len(h) % 2 != 0:
				h = h + '0'
			return h
		else:
			return h

	def wait_for_packet(self, timeout=1000):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 53 bytes as a packet.
		:param timeout how many bytes to attempt before stopping and giving up, returning False
		:param verbose Set to true to print out all the extra bytes that were read in before receiving a star byte or timing out
		:return True if the program can read in the next 53 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		data = ''
		for i in range(timeout):
			#Check for initialization byte
			try:
				dat = self.port.read(1) 
				if dat == b'\xAA':
					#Read next 2 bytes (counter and crc8)
					byte = self.readBytes(2)
					if byte != None and byte != []:
						counter = byte[0]
						crcVal = byte[1]
					else:
						continue

					#Combine initialization byte and counter
					p = [INIT_BYTE, counter]

					#Test CRC8 checksum
					return crc.crc8(p, table=self.parent.crc8_table) == crcVal
			except:
				print('Communication failing. Replug USB')
				continue

	def __check_crc32(self, crc_val, p):
	    """
	    Check crc Checksum with 32 bits
	    :param crc the 32 bit checksum as a list of bytes (ints) or an int
	    :param p the list of bytes to compare to the checksum. (This is bytes 0 to 46 in the 53 byte sensor packet)
	    :param polynomial the bit string of the crc polynomial to use
	    	Default is 0x04C11DB7 which is what we use for n=32
	    :return True if the checksum matches, False otherwise
	    """
	    if type(p) == int:
	    	p = self.toHex(p)
	    	p = self.toBytesList(p)
	    if type(crc_val) != int:
	    	crc_val = self.toInt(crc_val)

	    checksum = crc.crc32(p, table=self.parent.crc32_table)

	    return checksum == crc_val

	def toInt(self, byte, lsb_first=False):
		"""
		Helper method to convert a list of bytes where the least significant byte is 
		first or last into an int
		:param byte the list of bytes to convert to an int
		:param lsb_first True if the least significant byte of the number is listed first, False otherwise
			Default is false, so Most Significant Byte is assumed to be first
		"""
		num = 0
		sz = len(byte)

		if lsb_first: #LSB is first
			for i in range(sz):
				num += (byte[i] << i*8)
		else: #LSB is last
			for i in range(sz):
				num += (byte[sz - 1 - i] << i*8)

		return num

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

	def __parse(self, packet):
		"""
		Parse the data packet of 53 bytes using the SH-2 structure
		:return sensor_output object which contains the parsed data in a useful form
			returns a sensor_output object containing only the original received packet
				if the packet length is not 53 bytes
			returns None if the crc checksum did not match
		"""

		if not self.__check_crc32(packet[49:],packet[:49]):
			return out.sensor_output()
			print('Checksum Failed')

		rid = 'None'

		try:
			rid = self.rep_ids.get(packet[36])
		except KeyError:
			print('Invalid report ID key')

		return out.sensor_output(self.parent, packet, rid, self.parent.calMatrix)

	def run(self):
		"""
		What this thread actually does. This is executed when thread.start() is called. It should read
		data from serial and save it to the global data queue so the main thread can use it. The thread 
		ends if exitFlag is set to True, which happens when stop_data_transfer() is called
		"""
		global data_stream
		while (True): #Run until exitFlag is set
			if self.exitFlag:
				break

			#Read 53 bytes, ensuring they are all from one package by finding the initialization byte first
			# try:
			do_read = self.wait_for_packet()

			if do_read: #If it didn't time out, go ahead with read/write
				self.threadLock.acquire()
				data = self.readBytes(53)
				if data != None and len(data) == 53: #If there was an error, ignore and move on
					parsed = self.__parse(data)
					self.data_source.emit(parsed)
					data_stream = parsed
					self.numAdded += 1
					self.threadLock.release()
				else:
					print('Failed to read a packet')

