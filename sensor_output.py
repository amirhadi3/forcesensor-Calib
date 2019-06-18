class sensor_output:
	"""
	Class used to store parsed sensor outputs. The following data is included:

	differential : Array of 6 differential values
	sums : array of 6 sum values
	report_id : The report ID of the IMU section of the packet (int)
	sequence_num : The Sequence Number (int)
	accuracy : An accuracy value returned by the sensor. (0=Unreliable, 1=Low, 2=Medium, 3=High)
	delay : The report delay in seconds
	imu : Three component vector returned by the IMU (e.g. acceleration or rotation vector, etc.)
	report_id_str : The topic of the IMU report, given by the report ID. (e.g. acceleration or rotation_vector)
	checksum : The 32bit checksum returned by the sensor for error checking (Expressed as an int)
	byte : The actual unparsed package of bytes

	If the packet is of different length than 51 bytes, only the byte parameter will be initialized
	since in this case it cannot be parsed according to our packet structure
	"""
	def __init__(self, byte=None, type_str=None):
		if byte == None:
			self.byte = None
			self.differential = None
			self.sums = None
			self.report_id = None
			self.sequence_number = None
			self.accuracy = None
			self.delay = None
			self.imu = None
			self.report_id = None
			self.checksum = None
		elif len(byte) != 51: #Undefined length of data - don't know what to do with it
			print('Error in reading from sensor: unexpected output size')
			self.byte = byte
			self.differential = None
			self.sums = None
			self.report_id = None
			self.sequence_number = None
			self.accuracy = None
			self.delay = None
			self.imu = None
			self.report_id = None
			self.checksum = None
		else:
			#Convert all 6 values in differentials and sums to ints
			self.differential = [self.to_int(byte[i:i+3]) for i in {0, 3, 6, 9, 12, 15}]
			self.sums = [self.to_int(byte[i:i+3]) for i in {18, 21, 24, 27, 30, 33}]
			
			self.report_id = byte[37]
			self.sequence_num = byte[38]
			status = '{0:08b}'.format(byte[39])
			self.accuracy = int(status[-2:],2) #0=Unreliable, 1=Low, 2=Medium, 3=High
			self.delay = (byte[40] + int(status[0:-2] + '0000000',2)) * 100e-6 #delay in seconds

			#LSB is first byte that is returned
			x = self.to_int(byte[41:43])
			y = self.to_int(byte[43:45])
			z = self.to_int(byte[45:47])
			self.imu = {x,y,z}

			self.report_id_str = type_str if type_str != None else 'invalid_report_id'

			#Again, LSB comes first (?)
			#self.checksum = byte[47:]
			self.checksum = self.to_int(byte[47:])

			self.byte = byte

	def to_array(self):
		"""
		Return array representation of the measurement in the following order:
		<report_id, sequence_num, accuracy, delay, checksum, differential, sums, imu>
		Where differential and sums each have 6 components, imu has 3 components, and report_id, sequence_num
		are expressed as ints
		"""
		return {report_id, sequence_num, accuracy, delay, checksum, differential, sums, imu}

	def to_int(self, byte):
		"""
		Helper method to convert a bytes object where the LSB is first into an int
		"""
		num = sum([byte[i] * 16**(i*2) for i in range(len(byte))])
		return num

	def print(self):
		"""
		Print the data contained in the output object in a somewhat useful manner
		"""
		if self.byte == None:
			print('Sensor Output is broken')
			return
		if self.checksum == None:
			print(self.byte.hex())
			return
		print('Differential: ' + str(self.differential))
		print('Sum: ' + str(self.sums))
		print(self.report_id_str + ' (' + hex(self.report_id) + ') : ' + str(self.imu))
		print('Sequence Number: ' + str(self.sequence_num))
		print('Accuracy: ' + str(self.accuracy))
		print('Delay: ' + str(self.delay))