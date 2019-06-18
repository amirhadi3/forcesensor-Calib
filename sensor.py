import numpy as np
import sensor_output as out
import ftd2xx
import queue
import threading
import time

INIT_BYTE = 0xFF

"""
Implementation of sensor class using ftd2xx instead of pyftdi.serialext.
This is probably the best because the latency timer can be set programmatically
To use, initialzie a sensor object, send commands and receive sensor output by calling
the functions below
"""
# data_stream = queue.Queue()
data_stream = None
class sensor:
	
	def __init__(self, baud=5000000, portNum=0):
		"""
		Opens a connection to a serial device and creates a serial object with which to control it
		:param portNum Index of port of device to open. Default value is 0, but 1 or even 2
			are also common
		:param baud baudrate of connection. Ensure this matches sensor
		"""

		self.port = ftd2xx.open(portNum)
		self.port.setLatencyTimer(1)
		self.port.setTimeouts(16,16)
		self.port.setUSBParameters(64)
		self.port.setBaudRate(baud)
		self.__reset()
		self.threadLock = threading.Lock()
		self.thread = read_thread(port=self.port, threadLock=self.threadLock)


	#Clear memory and close session
	def __del__(self):
		self.port.close()
		print('Connection closed')


	def start_data_transmission(self):
		"""
		Start continuous transmission of data from the sensor by starting a thread
		that reads continuously without blocking and writes to a queue
		"""
		self.__reset()
		#self.reset() #This breaks everything for some reason
		#time.sleep(0.001)
		self.__do(b'\x10')
		self.thread.start()


	def stop_data_transmission(self):
		"""
		Stop continuous transmission of data from the sensor by ending the thread
		and clearing all associated buffers to avoid receiving extra data after the fact.
		The data stream remains untouched.
		"""

		# Keep telling device to shut up until it finally does
		self.__do(b'\x11')
		self.thread.exitFlag = True #End the thread

		while self.port.getStatus()[0] > 0:
			self.__do(b'\x11')
			self.reset() #Reset device's buffers
			self.__reset() #Reset computer's buffers
			time.sleep(0.02) #Wait to see if new data comes in
			self.thread.exitFlag = True #End the thread
			
		if self.thread.is_alive():	
			self.thread.join(1)
		

	def read_packet(self):
		"""
		Read and parse 51 bytes from the continuous data stream.
		This should be called in a loop after calling start_data_transmission()
		:return sensor_output object of the data
			returns Empty sensor_output if it was unable to read 51 bytes from the stream
		"""
		global data_stream

		# try:
		# 	data = data_stream.get(timeout=0.05) #Get one measurement, removing it from the data stream. Wait up to 5ms before timing out
		# except queue.Empty:
		# 	print('Error: Error reading from stream (empty)')
		# 	return out.sensor_output()
			
		if data_stream != None:
			data = data_stream
		else:
			return out.sensor_output()

		if len(data) != 51:
			print('Error: Error reading from stream (!=51)')
			return out.sensor_output()

		return self.__parse(data)


	def request_data(self):
		return self.__do(b'\x12', return_packet=True)
	def reset(self):
		self.__do(b'\xF0')
	def reset_imu(self):
		self.__do(b'\xFA')
	def reset_dac(self):
		self.__do(b'\xFB')
	def reset_transducer(self, index):
		if index <= 6 and index > 0: 
			self.__do(b'\xf0',index=index)
		else: print('Invalid index')

	#Dictionary of report IDs
	rep_ids = {
		b'\x01': 'Accelerometer',
		b'\x02': 'Gyroscope',
		b'\x04': 'Linear Acceleration',
		b'\x05': 'Rotation Vector',
		b'\x08': 'Game Rotation Vector'
		}


	def __crc32(self, crc, p, length=47):
		"""
		Check CRC32 Checksum
		:param crc the 4 byte checksum in hex or int - note that the sensor returns LSB first, 
			so can't just use sensor output here
		:param p the packet to compare to the checksum. This is bytes 0 to 46 in the 51 byte sensor packet
		:param length the length of the packet. In this application should generally be 47
		:return crc (the original checksum value) if it matches. Some other 32 bit number otherwise
		"""
		crc = 0xffffffff & ~crc
		for i in range(length):
			crc = crc ^ p[i]
			for j in range(8):
				crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1))
		return 0xffffffff & ~crc

	#How I would implement this based on Wikipedia...
	def __check_crc(self, crc, p, n=32, polynomial=0xedb88320):
	    """
	    Check CRC Checksum with arbitrary number of bits
	    :param crc the n bit checksum in hex or int - note that the sensor returns LSB first, 
	        so can't just use sensor output here
	    :param p the packet to compare to the checksum. (This is bytes 0 to 46 in the 51 byte sensor packet)
	    :param polynomial the bit string of the CRC polynomial to use
	    :return True if the checksum matches, False otherwise
	    """
	    
	    #Convert p to correct type
	    if type(p) == bytes:
	   		p = int(p.hex(),16)

	    #Construct the number: <4 bit counter><init byte><4 bit crc>
	    p = (p << n) + crc #Append the crc to the end of the number
	    pBin = bin(p)[2:] #Store the binary representation of the number
	    length = len(pBin)
	    poly = polynomial << length - len(bin(polynomial))+2 #Shift the polynomial to align with most significant bit
	    minVal = 2**n #When p gets smaller than this, the dividend is zero
	    i = 0 #Start aligned with most significant bit

	    while p >= minVal: #Terminate when the dividend is equal to zero and only the checksum portion remains
	        while pBin[i] == '0' and i <= length-n: #Shift the divisor until it is aligned with the most significant 1
	            i = i + 1
	            poly = poly >> 1
	        p = p ^ poly #XOR the number with the divisor
	        pBin = bin(p)[2:] #Update the bit string for checking
	        #Make sure leading zeros weren't removed by Python
	        while len(pBin) < length and pBin != '0':
	            pBin = '0' + pBin
	    return p == 0

	def __reset(self):
		"""
		Reset all input and output buffers until there are no more bytes waiting.
		If the buffer keeps filling up with new data after the reset, this will try
		100 times to clear the buffers before it gives up because something is continuosly
		writing to the buffer
		"""
		i = 0
		while self.port.getQueueStatus() > 0 and i < 100:
			self.port.resetDevice()
			i += 1

		if i == 100:
			print('Failed to reset. Ensure nothing is actively sending data')


	def __parse(self, packet):
		"""
		Parse the data packet of 51 bytes using the SH-2 structure
		:return sensor_output object which contains the parsed data in a useful form
			returns a sensor_output object containing only the original received packet
				if the packet length is not 51 bytes
			returns None if the crc checksum did not match
		"""
		#return out.sensor_output(packet, self.rep_ids.get(packet[36]))
		crc = self.__to_int(packet[47:])
		if not self.__check_crc(crc,packet[:47]):
			print('Checksum failed')
		return out.sensor_output(packet, self.rep_ids.get(packet[36])) if (self.__check_crc(crc,packet[:47])) else out.sensor_output()

	def __start_read(self, byte):
		"""
		Having found an initialization byte, test that the following byte contains 
		the correct counter and checksum 
		:param byte the byte that follows the initialization byte
			If correct, this byte should be a 4-bit counter in the most significant 4 bits 
			and a 4 bit CRC checksum in the least significant 4 bits 
		:return True if the checksum matches the counter and initialization byte and the
			next 51 bytes should be read in as a packet
			False otherwise
		"""
		
		#The 4-bit polynomial bitstring 0x09 provides good differentiation between numbers 0-15
		poly_bits = 0x09

		#Store 4 LSBs of the byte (the checksum)
		crc = byte[0] & 0x0F

		#Combine initialization byte and counter by shifting 4 bits and adding
		p = (INIT_BYTE << 4) + ((byte[0] & 0xF0) >> 4)  #4 MSBs of byte are the counter
		#Check CRC checksum
		return self.__check_crc(crc,p,4,poly_bits)

	def __init_read(self, timeout=1000):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 51 bytes as a packet.
		:param timout how many bytes to attempt before stopping and giving up, returning False
		:return True if the program can read in the next 51 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		#Iterate until timeout
		for i in range(timeout):
			#Check for initialization byte
			if self.port.read(1) == bytes([INIT_BYTE]):
				#Test checksum
				if self.__start_read(self.port.read(1)):
					return True
		return False

	def __do(self, task, index=0, return_packet=False):
		"""
		Send an arbitrary command given by task and potentially read the response.
		This method is thread safe and acquires/releases a lock around its read/write operations
		so the serial communication does not get messed up
		:param task byte to send to device
		:param index number to add to task byte. E.g. reset_transducer requires an index to say which transducer
		:param return_packet decide whether to read in and parse a 51 byte packet after sending the command
		:return parsed response from sensor if return_packet is True. None otherwise
		"""
		
		#Write the command, ensuring it is a bytes object, and that no other thread is trying to write at this time
		self.threadLock.acquire()
		print('Sending ' + str(bytes([int(task.hex(),16)+index])))
		self.port.write(bytes([int(task.hex(),16)+index]))

		#Make sure everything gets sent.This is very important as it WILL randomly fail to send otherwise.
		#However, if the device has started sending bytes back already, this could mess up the transmission if purge
		#also affects the receive buffer. mask=1 or 0 clears receive buffer, which could be bad here unless it happens
		#fast enough that the sensor has not had time to respond yet
		self.port.purge(mask=1) 
		
		#Read in, parse, and return the response
		if return_packet:
			do_read = self.__init_read()
			if do_read: #If it didn't time out, read in 51 byte packet
				data = self.port.read(51)
				self.threadLock.release()
				return self.__parse(data) if len(data) == 51 else out.sensor_output()
			else: #If it did time out, return empty sensor output
				self.threadLock.release()
				return out.sensor_output()
		else:
			self.threadLock.release()
			return None

	def __to_int(self, byte):
		"""
		Helper method to convert a bytes object where the least significant byte is first into an int
		"""
		num = 0
		for i in range(len(byte)):
			num += (byte[i] << i*8)
		return num

class read_thread (threading.Thread):
	"""
	Class that inherits threading and defines the behaviour of a separate thread to read data
	continuously without blocking the main thread. To start this thread, first initiate it
	using the constructor thread = read_thread(port, lock). Then use thread.start(), which executes
	the run function in a new thread
	"""
	def __init__(self, port, threadLock, threadID=0, name='read_thread'):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.port = port
		self.threadLock = threadLock
		self.exitFlag = False
		self.sensor = sensor
		self.numAdded = 0

	def __init_read(self, timeout=1000):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 51 bytes as a packet.
		:param timout how many bytes to attempt before stopping and giving up, returning False
		:return True if the program can read in the next 51 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		#Iterate until timeout
		for i in range(timeout):
			#Check for initialization byte
			if self.port.read(1) == bytes([INIT_BYTE]):
				#Read next byte
				byte = self.port.read(1)

				#Store 4 LSBs of the byte (the checksum)
				crc = byte[0] & 0x0F

				#Combine initialization byte and counter by shifting 4 bits and adding
				p = (INIT_BYTE << 4) + ((byte[0] & 0xF0) >> 4) #4 MSBs of byte are the counter

				#Test checksum
				return self.__check_crc(crc,p,4,0x9)
		return False

	def __check_crc(self, crc, p, n=32, polynomial=0xedb88320):
	    """
	    Check CRC Checksum with arbitrary number of bits
	    :param crc the n bit checksum in hex or int - note that the sensor returns LSB first, 
	        so can't just use sensor output here
	    :param p the packet to compare to the checksum. (This is bytes 0 to 46 in the 51 byte sensor packet)
	    :param polynomial the bit string of the CRC polynomial to use
	    :return True if the checksum matches, False otherwise
	    """
	    
	    #Convert p to correct type
	    if type(p) == bytes:
	   		p = int(p.hex(),16)

	    #Construct the number: <4 bit counter><init byte><4 bit crc>
	    p = (p << n) + crc #Append the crc to the end of the number
	    pBin = bin(p)[2:] #Store the binary representation of the number
	    length = len(pBin)
	    poly = polynomial << length - len(bin(polynomial))+2 #Shift the polynomial to align with most significant bit
	    minVal = 2**n #When p gets smaller than this, the dividend is zero
	    i = 0 #Start aligned with most significant bit

	    while p >= minVal: #Terminate when the dividend is equal to zero and only the checksum portion remains
	        while pBin[i] == '0' and i <= length-n: #Shift the divisor until it is aligned with the most significant 1
	            i = i + 1
	            poly = poly >> 1
	        p = p ^ poly #XOR the number with the divisor
	        pBin = bin(p)[2:] #Update the bit string for checking
	        #Make sure leading zeros weren't removed by Python
	        while len(pBin) < length:
	            pBin = '0' + pBin
	    return p == 0

	
	def run(self):
		"""
		What this thread actually does. This is executed when thread.start() is called. It should read
		data from serial and save it to the global data queue so the main thread can use it. The thread 
		ends if exitFlag is set to True, which happens when stop_data_transfer() is called
		"""
		global data_stream
		self.port.resetDevice()

		while (True): #Run until exitFlag is set
			if self.exitFlag:
				break

			#Read 51 bytes, ensuring they are all from one package by finding the initialization byte first
			try:
				do_read = self.__init_read()

				if do_read: #If it didn't time out, go ahead with read/write
					#data_stream.put(self.port.read(51)) #Write 51 bytes to global queue
					self.threadLock.acquire()
					data_stream = self.port.read(51)
					self.threadLock.release()
					self.numAdded += 1
					if self.numAdded == 1500:
						print('Done 1500\n\n\n\n\n\n\n\n\n\n')
						done = time.time()
						

			except ftd2xx.ftd2xx.DeviceError:
				print('Error. Closing read thread and stopping data transfer.')
				break #Stop if there is some error

		donedone = time.time()
		print(donedone-done)