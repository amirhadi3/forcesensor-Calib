from math import atan2, asin, copysign
from numpy import pi
import numpy as np

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

	If the packet is of different length than 53 bytes, only the byte parameter will be initialized
	since in this case it cannot be parsed according to our packet structure
	"""
	def __init__(self, sensor=None, byte_data=None, type_str=None, calMatrix=None):
		if byte_data == None:
			self.byte = None
			self.differential = None
			self.differential_raw = None
			self.sum = None
			self.sum_raw = None
			self.imu = None
			self.quaternion = None
			self.rotation = None
			self.report_id = None
			self.report_id_str = None
			self.checksum = None
			self.temperature = None
			self.wrench = None
			self.saturated = None
			self.saturatedChan = [0]*6

		else:
			if sensor != None:
				G = sensor.adsGain
				vref = sensor.vref
				a = []
				b = []
				for i in range(6):
					a.append(sensor.ab[sensor.adsRate[i]][0])
					b.append(sensor.ab[sensor.adsRate[i]][1])
				ofc = sensor.OFC
				fsc = sensor.FSC
				buf = sensor.inBuf
			else:
				G = [1]*6
				vref = 2.5
				a = [0x400000]*6
				b = [1.8639]*6
				ofc = [0]*6
				fsc = [0x44ac08]*6
				buf = [True]*6

			self.saturated = [0]*6

			#Convert all 6 values in differentials and sums to ints
			self.differential_raw = [self.to_int(byte_data[i:i+3]) for i in range(0,18,3)]
			
			#print(fsc)
			#print(ofc)
			self.differential = [self.volts_ads(self.differential_raw[i], G[i], vref, a[i], b[i], ofc[i], fsc[i]) for i in range(6)]

			self.sum_raw = [self.to_int(byte_data[i:i+3]) for i in range(18,36,3)]
			self.sum = [self.volts_adc(self.sum_raw[i],vref) for i in range(6)]

			for i in range(6):
				self.checkSaturation(self.differential[i],self.sum[i],i,buf[i],G[i])
			
			self.wrench = [0,0,0,0,0,0] if 0 in self.sum else [self.differential[i] / self.sum[i] for i in range(6)]
			self.wrench = np.matmul(calMatrix, self.wrench) 
			self.wrench = [self.wrench[i] for i in range(6)]
			self.byte = byte_data

			#Save report ID
			self.report_id_str = type_str
			rid = byte_data[36]
			self.report_id = rid

			#Configure Q Point based on report ID
			qpoint=0
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
				self.imu = self.imu[:4]
			else:
				quat = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), 14) for i in range(37,43,2)]
				quat.append(self.q_to_float(self.to_int(byte_data[43:45], lsb_first=True), 12))
				quat.append(self.q_to_float(self.to_int(byte_data[45:47], lsb_first=True), 12))
				self.imu = quat[:4]

			if rid == 5 or rid == 8:
				#Normalise if need be
				mag = 0
				for num in self.imu:
					mag += num**2
				mag = np.sqrt(mag)
				if mag > 1.1:
					self.imu = [self.imu[i] / mag for i in range(4)]

				self.quaternion = self.imu

				self.rotation = self.quaternionToRotation(self.imu)
			else:
				self.quaternion = None

			#Parse Temperature Data
			temp = self.to_int(byte_data[47:49])
			if byte_data[47] >= 2048: #The number begins with 1 and is thus negative
				temp -= 2**12 #2's complement
			self.temperature = temp * 0.0625

			#LSB comes last
			self.checksum = self.to_int(byte_data[49:])

	def isSaturated(self):
		return sum(self.saturated) > 0

	def checkSaturation(self, Vdiff, Vsum, idx, buff, gain):
		"""
		Based on table 7.3 (pg. 6) of:
		http://www.ti.com/lit/ds/symlink/ads1257.pdf
		"""
		#Check absolute input voltage
		Vbb = 4.775
		Vainn = -Vsum + Vbb - 0.5*Vdiff
		Vainp = -Vsum + Vbb + 0.5*Vdiff

		if buff:
			minV = 0
			maxV = 3
		else:
			minV = -0.1
			maxV = 5.1

		if Vainn > maxV or Vainn < minV or Vainp > maxV or Vainp < minV:
			self.saturated[idx] = 1
			return

		#Check differential voltage
		if abs(Vdiff) > 5 / gain:
			self.saturated[idx] = 1
			return

		#Check sum voltage
		if abs(Vsum) > 2.5:
			self.saturated[idx] = 1


	def volts_ads(self, v, G, Vref, a, b, OFC, FSC):
		"""
		Find Vin from Vout according to the conversion given in Equation 4 (Pg. 34) of 
		http://www.ti.com/lit/ds/symlink/ads1257.pdf
		"""
		if v >= 2**23:
			v -= 2**24

		return (v/(b*FSC) + OFC/a) * (2*Vref/G)

	def volts_adc(self, v, Vref):
		"""
		Find Vin from Vout according to the conversion given in Equation 2 (Pg. 9) of 
		https://www.intel.com/content/dam/www/programmable/us/en/pdfs/literature/hb/max-10/ug_m10_adc.pdf
		"""
		return v * (Vref/2**12)

	def quaternionToRotation(self, q):
		qI = q[0]
		qJ = q[1]
		qK = q[2]
		qR = q[3]

		rm = np.zeros((3,3))

	  	rm[0,0] = 1 - 2 * qJ**2 - 2 * qK**2;
	    	rm[0,1] = 2 * qI * qJ - 2 * qR * qK;
	    	rm[0,2] = 2 * qI * qK + 2 * qR * qJ;
	    	rm[1,0] = 2 * qI * qJ + 2 * qR * qK;
	    	rm[1,1] = 1 - 2 * qI**2 - 2 * qK**2;
	    	rm[1,2] = 2 * qJ * qK - 2 * qR * qI;
	    	rm[2,0] = 2 * qI * qK - 2 * qR * qJ;
	    	rm[2,1] = 2 * qJ * qK + 2 * qR * qI; 
	    	rm[2,2] = 1 - 2 * qI**2 - 2 * qJ**2;

	    	return rm

	def failed(self):
		if self.sum == None:
			return True
		else:
			return False

	def to_array(self, detailed=False):
		"""
		Return array representation of the measurement in the following order:
		<report_id, sequence_num, accuracy, delay, checksum, differential, sums, imu>
		Where differential, wrench, and sums each have 6 components, imu has 3 components
		"""
		if detailed:
			return self.wrench + self.differential + self.sum + self.imu + [self.temperature]
		else:
			return self.wrench + self.imu + [self.temperature]

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

	def string(self, detailed=False):
		"""
		Print the data contained in the output object in a useful manner
		"""
		if self.byte == None:
			print('Sensor Output is broken')
			return
		if self.checksum == None:
			print(self.byte.hex())
			return
		if self.isSaturated():
			print('Caution, channels ' + str(self.saturated) + ' are saturated')
		print('Wrench: ' + str(self.wrench))
		print(str(self.report_id_str) + ' (' + hex(self.report_id) + ') : ' + str(self.imu))
		print('Temperature: ' + str(self.temperature))

		if detailed:
			print('Differential (Volts): ' + str(self.differential))
			print('Differential (Raw) ' + str(self.differential_raw))

			print('Sum (Volts): ' + str(self.sum))
			print('Sum (Raw) ' + str(self.sum_raw))
		print('\n')
