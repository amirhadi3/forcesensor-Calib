class sensor_output:
	"""
	Class used to store parsed sensor outputs. The following data is included:

	differential : Array of 6 differential values
	sum : array of 6 sum values
	report_id : The report ID of the IMU section of the packet (int)
	imu : Three component vector returned by the IMU (e.g. acceleration or rotation vector, etc.)
	report_id : The topic of the IMU report, given by the report ID. (e.g. acceleration or rotation_vector)
	checksum : The 32bit checksum returned by the sensor for error checking (Expressed as an int)
	byte : The actual unparsed package of bytes

	If the packet is of different length than 51 bytes, only the byte parameter will be initialized
	since in this case it cannot be parsed according to our packet structure
	"""
	def __init__(self, byte_data=None, type_str=None):
		if byte_data == None:
			self.byte = None
			self.differential = None
			self.sum = None
			self.imu = None
			self.report_id = None
			self.report_id_str = None
			self.checksum = None

		elif len(byte_data) != 51: #Undefined length of data - don't know what to do with it
			print('Error in reading from sensor: unexpected output size')
			self.byte = byte_data
			self.differential = None
			self.sum = None
			self.imu = None
			self.report_id = None
			self.report_id_str = None
			self.checksum = None

		else:
			#Convert all 6 values in differentials and sums to ints
			self.differential = [self.to_int(byte_data[i:i+3]) for i in range(0,18,3)]
			self.sum = [self.to_int(byte_data[i:i+3]) for i in range(18,36,3)]
			self.byte = byte_data

			#Save report ID
			self.report_id_str = type_str
			rid = byte_data[36]
			self.report_id = rid

			#Configure Q Point based on report ID
			if rid == 1 or rid == 4:
				qpoint = 8
			elif rid == 2:
				qpoint = 9
			elif rid == 8:
				qpoint = 14

			#Parse IMU data
			#LSB is first byte that is returned
			if rid != 5:
				self.imu = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), qpoint) for i in range(37,47,2)]
			else:
				quat = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), 14) for i in range(37,43,2)]
				quat.append(self.q_to_float(self.to_int(byte_data[43:45], lsb_first=True), 12))
				quat.append(self.q_to_float(self.to_int(byte_data[45:47], lsb_first=True), 12))
				self.imu = quat

			#LSB comes last
			self.checksum = self.to_int(byte_data[47:])

	def to_array(self):
		"""
		Return array representation of the measurement in the following order:
		<report_id, sequence_num, accuracy, delay, checksum, differential, sums, imu>
		Where differential and sums each have 6 components, imu has 3 components, and report_id, sequence_num
		are expressed as ints
		"""
		return {report_id, sequence_num, accuracy, delay, checksum, differential, sums, imu}

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

	def q_to_float(self, qInt, qPoint):
		"""
		Convert the Q-Point Number in qInt with its decimal point at position qPoint
		(counting from the LSB, starting at 0) into a floating point decimal number
		"""
		return float(qInt) * 2**(-qPoint)

	def string(self):
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
		print('Sum: ' + str(self.sum))
		print(self.report_id_str + ' (' + hex(self.report_id) + ') : ' + str(self.imu))
		print('IMU Output: ' + str(self.imu))