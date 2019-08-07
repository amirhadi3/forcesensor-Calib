import numpy as np
import sensor_output as out
import pylibftdi as ftdi
import CRC
import threading
import time
import struct

INIT_BYTE = 0xAA
BYTES_PER_DIGIT_IN = 1
"""
Implementation of sensor class using ftd2xx instead of pyftdi.serialext.
This is probably the best because the latency timer can be set programmatically
To use, initialzie a sensor object, send commands and receive sensor output by calling
the functions below
"""

# data_stream = queue.Queue()
data_stream = None
class sensor:
	
	def __init__(self, baud=5000000, portNum=1):
		"""
		Opens a connection to a serial device and creates a serial object with which to control it
		:param portNum Index of port of device to open. Default value is 1, but 2 is also common
		:param baud baudrate of connection. Ensure this matches sensor
		"""

		#Dictionary of report IDs
		self.rep_ids = {
			1: 'Accelerometer',
			2: 'Gyroscope',
			4: 'Linear Acceleration',
			5: 'Rotation Vector',
			8: 'Game Rotation Vector'
			}

		self.crc4_table = CRC.calculate_CRC4_table()
		self.crc32_table = CRC.calculate_CRC32_table()

		self.num_errors = 0

		self.port = ftdi.Device('USB-COM485 Plus2', interface_select=portNum)
		self.port.ftdi_fn.ftdi_set_latency_timer(1)
		self.port.ftdi_fn.ftdi_set_line_property(8,1,0)
		# self.port.setTimeouts(16,16)
		# self.port.setUSBParameters(64)
		self.port.baudrate = baud
		#self.__reset()
		self.threadLock = threading.Lock()
		self.thread = read_thread(crc4_table=self.crc4_table, port=self.port, threadLock=self.threadLock)

	#Close session
	def __del__(self):
		try:
			self.port.close()
		except AttributeError:
			print('Shutting down')
		print('Connection closed')
		

	def read(self, timeout=10000):
		"""
		Read and parse 51 bytes from the continuous data stream.
		This should be called in a loop after calling start_data_transmission()
		:return sensor_output object of the data
			returns Empty sensor_output if it was unable to read 51 bytes from the stream
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
			data = data_stream
			data_stream = None #Get rid of the old measurement

		if len(data) != 51:
			print('Error: Error reading from stream (!=51)')
			return out.sensor_output()

		return self.__parse(data)

	####### Functions of the sensor  #######
	###  Measurements  ###
	def start_data_transmission(self, data_rate):
		"""
		Start continuous transmission of data from the sensor by starting a thread
		that reads continuously without blocking and writes to a queue
		"""
		
		data_rate = self.to_hex(data_rate)
		while len(data_rate) < 3:
			data_rate = '0' + data_rate

		byte1 = int(data_rate[:2],16)
		byte2 = int(data_rate[2:3],16)

		#Prompt the start of continuous data transmission
		self.port.write(self.toStr([16, byte1, byte2 << 4], with_crc4=2))
		self.port.flush(ftdi.FLUSH_OUTPUT)

		self.thread.start()


	def stop_data_transmission(self):
		"""
		Stop continuous transmission of data from the sensor by ending the thread
		and clearing all associated buffers to avoid receiving extra data after the fact.
		The data stream remains untouched.
		"""

		# Tell device to stop
		self.__do([11])
		self.thread.exitFlag = True #End the thread
			
		if self.thread.is_alive():	
			self.thread.join(1)

	def poll(self):
		#Take one measurement and parse it
		return self.__do([0x12],expect_package=True)
	
	###  Reset and Deactivation  ###

	def reset_device(self):
		#Reset the device
		self.__do([0xF0])
	
	def reset_imu(self):
		#Reset the IMU
		self.__do([0xFA])
	
	def reset_dac(self):
		#Reset the DAC
		self.__do([0xFB])
	
	def reset_transducer(self, transducer_num):
		"""
		Reset and activate one of the ADS1257 transducers
		:param transducer_num the index of the transducer to reset
			enter 7 to reset all of them
		"""
		if transducer_num <= 6 and transducer_num > 0: 
			self.__do([0xF0 + transducer_num])
		elif transducer_num == 7: #Reset every transducer
			for i in range(6):
				self.__do([0xF0 + i])
		else: print('Invalid index')
	
	def deactivate_transducer(self, transducer_num):
		"""
		Deactivate one of the ADS1257 transducers
		:param transducer_num the index of the transducer to deactivate
			enter 7 to deactivate all of them
		"""
		if transducer_num <= 6 and transducer_num > 0: 
			self.__do([0xE0 + transducer_num])
		elif transducer_num == 7: #Reset every transducer
			for i in range(1,7):
				self.__do([0xE0 + i])
		else: print('Invalid index')

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
		:param delay the interval between reports in ms
		"""
		byte = []

		if type(mode) == str:
			mode = mode.lower()
			
			if mode.startswith('acc'):
				byte = [0x21, delay]
			elif mode.startswith('gyr'):
				byte = [0x22, delay]
			elif mode.startswith('lin'):
				byte = [0x24, delay]
			elif mode.startswith('rot'):
				byte = [0x25, delay]
			elif mode.startswith('gam'):
				byte = [0x26, delay]
			else:
				print('Invalid IMU Configuration Mode')
				return
		elif type(mode) == int:
			byte = [0x20 + mode ,delay]
		else:
			return
		self.__do(byte)

	def set_imu_accelerometer(self, delay):
		self.config_imu(1, delay)

	def set_imu_gyroscope(self, delay):
		self.config_imu(2, delay)

	def set_imu_lin_accel(self, delay):
		self.config_imu(4, delay)

	def set_imu_rotation(self, delay):
		self.config_imu(5, delay)

	def set_imu_game_rot(self, delay):
		self.config_iu(8, delay)

	def config_adc(self, adc_num, category, setting):
		"""
		Configure the ADS1257-x device(s). These are ADCs with built in PGAs.
		:param adc_num specifies which of the six ADCs to control
			Indexing is 1-6. -1 targets every ADC at once
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
		:param sensor_num specifies which sensor this is done to. -1 indicates all sensors
		""" 
		if adc_num not in [-1,1,2,3,4,5,6]:
			print('Invalid ADC number. Values between 1 and 6 are permitted, or -1 to select all')
			return

		#Set the first byte
		byte = [0x3 + adc_num] 
		
		#Select which category to work with
		cat_dict = {'root' : 0, 'drate' : 1, 'pga' : 2, 'pos' : 3, 'neg' : 4}
		if type(category) == str: #Convert string inputs
			category = cat_dict.get(category.lower(), value=-1)
		if category < 0 or category > 4: #Check validity
			print('Invalid Category String')
			return

		if category == 0: #Configure Root Registers
			if (type(setting) != tuple and type(setting) != str) or len(setting) != 2 or setting[0] not in [0,1] or setting[1] not in [0,1]:
				print('Invalid setting. Please enter a tuple or string with 2 binary elements to configure root registers')
				return

			config = int('000' + str(setting[0]) + str(setting[1]) + '000',2)

		elif category == 1: #Configure Data Rate
			rates = np.array([2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1e3, 2e3, 3.75e3, 7.5e3, 15e3, 30e3])
			#Ensure we have an acceptable value
			if setting not in rates:
				diff = np.abs(rates - setting)
				setting = rates[np.argmin(diff)]
				print('Data rate value not permitted. Rounding to ' + str(setting))

			drate = self.to_hex(np.where(rates == setting))
			config = int('1' + drate, 16)

		elif category == 2: #Configure PGA Gain
			gains = np.array([1, 2, 4, 8, 16, 32, 64])
			
			if setting not in gains:
				diff = np.abs(gains - setting)
				setting = gains[np.argmin(diff)]
				print('Gain value not permitted. Rounding to ' + str(setting))

			gain = bin(np.where(gains == setting)[0][0])[2:]
			while len(gain) < 5:
				gain = '0' + gain

			config = int('010' + gain, 2)

		elif category == 3: #Configure Positive Channel
			if setting not in [0,1,2,3]:
				print('Please enter a channel between 0 and 4 (inclusive)')
				return
			config = int('3' + str(setting), 16)

		elif category == 4: #Configure Negative Channel
			if setting not in [0,1,2,3]:
				print('Please enter a channel between 0 and 4 (inclusive)')
				return
			config = int('4' + str(setting), 16)

		byte.append(config)
		self.__do(byte)

	def set_adc_drate(self, adc_num, data_rate):
		"""
		Convenience method that calls config_adc to set the data rate
		:param adc_num which ADC to do this to (1-6)
		:param data_rate the desired data rate. If this is not one allowed by config_adc,
			it will be rounded to the nearest allowed value
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		self.config_adc(adc_num, 1, data_rate)

	def set_adc_registers(self, adc_num, ACAL, IN_BUFF):
			"""
			Convenience method that calls config_adc to configure the root registers
			:param adc_num which ADC to do this to (1-6)
			:param ACAL 1 to enable ACAL, 0 to disable it
			:param IN_BUFF 1 to enable IN_BUFF, 0 to disable it
			:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
			"""
			self.config_adc(adc_num, 0, (ACALL, IN_BUFF))

	def set_pga_gain(self, pga_num, gain):
		"""
		Convenience method that calls config_adc to set the PGA's gain
		:param pga_num which PGA to do this to (1-6)
		:param gain the desired gain value. If this is not one allowed by config_adc,
			it will be rounded to the nearest allowed value
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		self.config_adc(pga_num, 2, gain)

	def set_adc_channel(self, adc_num, channel, positive=True):
		"""
		Convenience method that calls config_adc to set the positive or negative channel number
		:param adc_num which ADC to do this to (1-6)
		:param positive
		:param channel the desired channel number (0-3)
		:param positive flag that states whether to change the positive or negative channel
			True changes the positive channel, false does the negative one
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		if positive:
			self.config_adc(adc_num, 3, channel)
		else:
			self.config_adc(adc_num, 4, channel)

	def config_dac(self, channel, voltage):
		"""
		Configure the DAC by setting the output voltage of a specified channel or powering off a channel
		:param channel the channel to set the voltage of (int between 1 and 6, inclusive)
		:param voltage the voltage to set the output of the selected channel to. This is a 12-bit number between 0 and max (0xFFF=4095)
			input 0 to shut a channel off
		"""
		
		if channel < 1 or channel > 6:
			print('Invalid channel number. Input a number between 1 and 7, inclusive')
			return

		config = []
		if voltage == 0:
			config = [0x47, channel]
		else:
			volts_bin = bin(voltage)[2:]

			#Adjust the length to 12 bits
			while len(volts_bin) < 12:
				volts_bin = '0' + volts_bin

			config = [(4<<4) + channel, int(volts_bin[:8],2), int(volts_bin[8:], 2) << 4]
		self.__do(config, with_crc4=2)

	def turn_on_led(self, ledNum):
		config_dac(ledNum, 0x666)
	def turn_off_led(self, ledNum):
		config_dac(ledNum, 0)

	#########   Data transmission #######

	def __do(self, task, print_garbage=False, expect_package=False, with_crc4=1):
		"""
		Send an arbitrary command given by task and potentially read the response.
		This method is thread safe and acquires/releases a lock around its read/write operations
		so the serial communication does not get messed up
		:param task byte to send to device
		:param index number to add to task byte. E.g. reset_transducer requires an index to say which transducer
		:param return_packet decide whether to read in and parse a 51 byte packet after sending the command
		param with_crc4 indicates whether to append a CRC4 to the message (1), or not (0), or append one in the last 4 bits
			of the final byte rather than as an extra byte (2)
		:return parsed response from sensor if return_packet is True. None otherwise
		"""
		
		#Write the command, ensuring it is a bytes object, and that no other thread is trying to write at this time
		self.threadLock.acquire()
		toWrite = self.toStr(task, with_crc4=with_crc4)
		#print('Sending ' + toWrite)
		self.port.ftdi_fn.ftdi_usb_purge_rx_buffer() #Purge buffers of any garbage first
		self.port.write(toWrite)
		#self.port.flush(ftdi.FLUSH_OUTPUT) 
		
		timeout = 20 #Expect 11 + random leading and following zeros
		if expect_package:
			timeout = 100

		flag = self.wait_for_packet(timeout, print_garbage=print_garbage)
		if flag:
			data = self.readBytes(51)
			self.threadLock.release()
			return self.__parse(data) if (data != None and len(data) == 51) else out.sensor_output()
		else: #If it did time out, return empty sensor output
			self.threadLock.release()
			return None

	def wait_for_packet(self, timeout=100, print_garbage=False):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 51 bytes as a packet.
		:param timout how many bytes to attempt before stopping and giving up, returning False
		:return True if the program can read in the next 51 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		data = []
		for i in range(timeout):
			#Check for initialization byte
			dat = self.readBytes(1) 
			if dat == [INIT_BYTE]:
				#Print all the garbage that was read in
				if print_garbage and data != []:
					string = ''
					for b in data:
						byte = str(b)
						if len(byte) != 2:
							byte = '0' + byte
						string += byte
					print(string)

				#Read next byte
				byte = self.readBytes(1)
				if byte != None and byte != []:
					byte = byte[0]
				else:
					continue

				#Store 4 LSBs of the byte (the checksum)
				crc = byte & 0x0F

				#Combine initialization byte and counter by shifting 4 bits and adding
				p = (INIT_BYTE << 4) + ((byte & 0xF0) >> 4) #4 MSBs of byte are the counter

				#Test checksum
				return self.__check_crc(crc,p,4,0x3)
			else:
				if dat != [] and dat != None:
					data.append(dat[0])

		#Print all the garbage that was read in
		if print_garbage and data != []:
			string = ''
			for b in data:
				byte = str(b)
				if len(byte) != 2:
					byte = '0' + byte
				string += byte
			print(string)
		return False


	###########    Helper Methods    ###########

	def __check_crc(self, crc, p, n=32, polynomial=0x04C11DB7):
	    """
	    Check CRC Checksum with 4 or 32 bits
	    :param crc the n bit checksum as a list of bytes (ints) or an int
	    :param p the list of bytes to compare to the checksum. (This is bytes 0 to 46 in the 51 byte sensor packet)
	    :param polynomial the bit string of the CRC polynomial to use
	    	Default is 0x04C11DB7 which is what we use for n=32. For n=4, 0x03 is commonly used
	    :return True if the checksum matches, False otherwise
	    """
	    if type(p) == int:
	    	p = self.toBytesList(self.to_hex(p))
	    if type(crc) != int:
	    	crc = self.to_int(crc)

	    if n == 4:
	    	checksum = CRC.crc4(p, polynomial, self.crc4_table)
	    elif n == 32:
	    	checksum = CRC.crc32(p, polynomial, self.crc32_table)

	    return checksum == crc

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
		Parse the data packet of 51 bytes using the SH-2 structure
		:return sensor_output object which contains the parsed data in a useful form
			returns a sensor_output object containing only the original received packet
				if the packet length is not 51 bytes
			returns None if the crc checksum did not match
		"""

		if not self.__check_crc(packet[47:],packet[:47]):
			print('Checksum failed')
			return out.sensor_output()
		
		rid = 'None'

		try:
			rid = self.rep_ids.get(packet[36])
		except KeyError:
			print('Invalid report ID key')

		return out.sensor_output(packet, rid)

	def readBytes(self, num_bytes):
		try:
			read_in = self.port.read(num_bytes * BYTES_PER_DIGIT_IN)
		except ftdi._base.FtdiError:
			print('FTDI Error thrown while reading')
			self.num_errors += 1
			if self.num_errors == 100:
				raise ftdi._base.FtdiError('Failed too many times. Exiting...')
			return None
		try:
			retval = self.toBytesList(read_in)
			return retval
		except ValueError:
			return None

	def to_int(self, byte, lsb_first=False):
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

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

	def toStr(self, byte_list, with_crc4=0, format=0):
		"""
		Do the opposite of toBytesList. Convert a list of bytes (array of 8-bit ints in decimal)
		to a byte string in hex.
		:param byte_list the list of bytes (as decimal ints) to convert to a byte string
		:param with_crc4 1 if a CRC4 checksum for the number given by byte_list should be appended to the string
			Default is 0, so no CRC4 is computed or added. A value of 2 adds the CRC4 to the last 4 bits of the last byte
			rather than as an additional byte
		:param format can take the value 0 (bytes) or 1 (string))
			If format = 0, numbers will undergo the following conversion: 15 -> 0x0f -> b'\x0f' which is a bytes
			object useful for sending to the sensor. This works in Python 2.7 and 3
			If format = 1, numbers will be converted directly to a hex string: 15 -> '0f', which is actually 2 bytes b'\x30\x66'.
			These are the ASCII values of the characters. This is not useable for sending to a sensor.
		"""
		string = ''

		if with_crc4 == 2:
			#If yes, find the CRC4 first, then shift the last byte 4 bits and add the CRC4
			crc = CRC.crc4(self.shift4(byte_list), table=self.crc4_table)
			byte_list[-1] <<= 4
			byte_list[-1] += crc
			with_crc4 = 0 #Make sure we don't add another CRC4

		for i in range(len(byte_list)):
			if format == 0:
				string += self.toBytes(byte_list[i])
			else:
				string += self.to_hex(byte_list[i])

		if with_crc4 == 1:
			if format == 0:
				string += self.toBytes(CRC.crc4(byte_list, table=self.crc4_table))
			else:
				string += '0' + self.to_hex(CRC.crc4(byte_list, table=self.crc4_table))

		return string

	def to_hex(self, num):
		"""
		Convert to hex without the "0x" at the start or the random "L" at the end
		"""
		return "%x" % num

	def toBytes(self,byteList):
		"""
		The only reliable Python 2 and 3- compatible int-to-bytes conversion I could find 
		"""
		byte = ''
		for num in byteList:
			byte += struct.pack("B", num)
		return byte

	def shift4(self, byteList):
		"""
		Shift the number given as a list of bytes in decimal form by 4 bits to the right
		This is mostly used for CRC4 calculation when the CRC4 is appended to the last byte
		"""
		string = ''
		for num in byteList:
			string += self.to_hex(num)

		string = '0' + string[:-1]
		return self.toBytesList(string)

class read_thread (threading.Thread):
	"""
	Class that inherits threading and defines the behaviour of a separate thread to read data
	continuously without blocking the main thread. To start this thread, first initiate it
	using the constructor thread = read_thread(port, lock). Then use thread.start(), which executes
	the run function in a new thread
	"""
	def __init__(self, crc4_table, port, threadLock, threadID=0, name='read_thread'):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.port = port
		self.threadLock = threadLock
		self.exitFlag = False
		self.sensor = sensor
		self.numAdded = 0
		self.crc4_table = crc4_table

	def readBytes(self, num_bytes):
		try:
			read_in = self.port.read(num_bytes * BYTES_PER_DIGIT_IN)
		except ftdi._base.FtdiError:
			print('FTDI Error thrown while reading')
			self.error_count += 1
			if self.error_count == 100:
				raise ftdi._base.FtdiError('Failed too many times. Exiting...')
			return None
		try:
			retval = self.toBytesList(read_in)
			return retval
		except ValueError:
			return None

	def to_hex(self, num):
		return "%x" % num

	def wait_for_packet(self, timeout=100, print_garbage=False):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 51 bytes as a packet.
		:param timout how many bytes to attempt before stopping and giving up, returning False
		:return True if the program can read in the next 51 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		data = []
		for i in range(timeout):
			#Check for initialization byte
			dat = self.readBytes(1) 
			if dat == [INIT_BYTE]:
				#Print all the garbage that was read in
				if print_garbage and data != []:
					string = ''
					for b in data:
						byte = str(b)
						if len(byte) != 2:
							byte = '0' + byte
						string += byte
					print(string)

				#Read next byte
				byte = self.readBytes(1)
				if byte != None and byte != []:
					byte = byte[0]
				else:
					continue

				#Store 4 LSBs of the byte (the checksum)
				crc = byte & 0x0F

				#Combine initialization byte and counter by shifting 4 bits and adding
				p = (INIT_BYTE << 4) + ((byte & 0xF0) >> 4) #4 MSBs of byte are the counter

				#Test checksum
				return self.__check_crc(crc,p)
			else:
				if dat != [] and dat != None:
					data.append(dat[0])

		#Print all the garbage that was read in
		if print_garbage and data != []:
			string = ''
			for b in data:
				byte = str(b)
				if len(byte) != 2:
					byte = '0' + byte
				string += byte
			print(string)
		return False

	def __check_crc(self, crc, p):
	    """
	    Check CRC Checksum with 4 or 32 bits
	    :param crc the n bit checksum as a list of bytes (ints) or an int
	    :param p the list of bytes to compare to the checksum. (This is bytes 0 to 46 in the 51 byte sensor packet)
	    :param polynomial the bit string of the CRC polynomial to use
	    	Default is 0x04C11DB7 which is what we use for n=32. For n=4, 0x03 is commonly used
	    :return True if the checksum matches, False otherwise
	    """
	    if type(p) == int:
	    	p = self.to_hex(p)
	    	p = self.toBytesList(p)
	    if type(crc) != int:
	    	crc = self.to_int(crc)

	    checksum = CRC.crc4(p, 0x3, self.crc4_table)

	    return checksum == crc

	def to_int(self, byte, lsb_first=False):
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

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

	def run(self):
		"""
		What this thread actually does. This is executed when thread.start() is called. It should read
		data from serial and save it to the global data queue so the main thread can use it. The thread 
		ends if exitFlag is set to True, which happens when stop_data_transfer() is called
		"""
		global data_stream
		self.port.ftdi_fn.ftdi_usb_reset()
		done = 0

		while (True): #Run until exitFlag is set
			if self.exitFlag:
				break

			#Read 51 bytes, ensuring they are all from one package by finding the initialization byte first
			try:
				do_read = self.wait_for_packet()

				if do_read: #If it didn't time out, go ahead with read/write
					#data_stream.put(self.port.read(51)) #Write 51 bytes to global queue
					self.threadLock.acquire()
					data = self.readBytes(51)
					if data != None: #If there was an error, ignore and move on
						data_stream = data
						self.numAdded += 1
					self.threadLock.release()
					if self.numAdded == 1500:
						print('Done 1500\n\n\n\n\n\n\n\n\n\n')
						done = time.time()
						

			except ftd2xx.ftd2xx.DeviceError:
				print('Error. Closing read thread and stopping data transfer.')
				break #Stop if there is some error

		donedone = time.time()
		print(donedone-done)