import comedi
import numpy as np
import xml.etree.cElementTree as xml

class Ati:

	def __init__(self, path, serialNum="FT24092", comediNum=0):
		#The defaults are configured for the Linux computer by the MTMs
		#The other force sensor is serialNum="FT24093", comedi_num=1
		self.num_channels = 6
		self.sub_device = 0
		self.aref = 0 # = AREF_GROUND
		self.range = 0 #0 = [-10V, 10V]
	
		#Create a comedi connection for connected device
		self.dev = comedi.comedi_open("/dev/comedi" + str(comediNum))
		#Read in calibration matrix
		self.cal_matrix = self.read_cal_file(path + "/calibration/" + serialNum + ".cal")
		self.bias = self.findBias()

		print("Created sensor " + serialNum + " on port " + str(comediNum) + "\n")

	def __del__(self):
		comedi.comedi_close(self.dev)

	def read_cal_file(self, filename):
		"""
		Read the .cal file into a calibration matrix
		"""
		root = xml.parse(filename).getroot()
		axes = root.findall('Calibration/UserAxis')
		return np.array([[float(axes[i].attrib['values'].split()[j]) for j in range(self.num_channels)] for i in range(self.num_channels)])

	def findBias(self):
		"""
		Find the wrench with which to bias all upcoming measurements. This assumes the sensor is
		fully unloaded
		:return the bias wrench
		"""
		print("Calibrating ATI F/T Sensor...\n")

		#Average num_samples measurements to remove outliers
		num_samples = 100
		avg = np.zeros(self.num_channels)
		for i in range(num_samples):
			dat = np.array(self.read_raw())
			avg += dat
		avg /= num_samples

		print("Calibration Successful\n")

		return avg

	def read(self):
		"""
		Read one sample from the connected sensor. Convert the reading to force and torque in N
		:return measured, calibrated, biased, wrench
		"""

		data_array = self.read_raw() #Measure & calibrate
		# data_array -= self.bias #Bias
		return data_array

	def read_raw(self):
		"""
		Read directly from 6 channels of the sensor and convert from uint-16 to voltage with 
		no additional processing
		"""
		data_array = np.empty(self.num_channels)
		for chan in range(self.num_channels):
			#Read in data
			rc, data_array[chan] = comedi.comedi_data_read(self.dev, self.sub_device, chan, self.range, self.aref)
			#Convert unsigned 16-bit ints to voltages
			data_array[chan] = self.comedi_to_phys(data_array[chan])
		
		#Multiply voltages by calibration matrix to get force/torque
		data = -np.matmul(self.cal_matrix, data_array)

		return data

	def comedi_to_phys(self, data):
		"""
		Convert measured uint-16 to voltage
		"""
		return (data - 2**15) / 2.0**15 *10