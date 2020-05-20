import numpy as np
from sensor import Sensor 
from datetime import datetime as date
import xml.etree.cElementTree as xml
import signal
import sys
import time

class MainObject:
	def __init__(self):
		signal.signal(signal.SIGINT, self.signal_handler)
		filename = './calibration.cal'
		root = xml.parse(str(filename)).getroot()
		axes = root.findall('Calibration/Axis')
		calMatrix = np.array([[float(axes[i].attrib['values'].split()[j]) for j in range(6)] for i in range(6)])

		sensor = Sensor(calMatrix)

		self.time = []
		self.sums = []
		self.diff = []
		self.temp = []

		ma = 0
		sumMax = 0
		start = time.time()
		try:
			while sumMax < 2.386:
				sample = sensor.poll()
				if sample != None:
					self.time.append(time.time()-start)
					self.temp.append(sample.temperature)
					self.sums.append(sample.sum)
					self.diff.append(sample.differential)

					if sample.isSaturated():
						print('Saturated')
						break

					sumMax = max(self.sums[-1])
					# print('Max sum value: ' + str(sumMax))
					# print('Current: ' + str(ma) + 'mA')
					sensor.config_dac(-1,ma)
					ma += 0.1
		finally:
			np.savetxt('./times.txt',np.array(self.time))
			np.savetxt('./temps.txt', np.array(self.temp))
			np.savetxt('./sum.txt', np.array(self.sums))
			np.savetxt('./diff.txt', np.array(self.diff))

	def signal_handler(self, sig, frame):
		print('Killing Application...')
		np.savetxt('./times.txt',np.array(self.time))
		np.savetxt('./temps.txt', np.array(self.temp))
		np.savetxt('./sum.txt', np.array(self.sums))
		np.savetxt('./diff.txt', np.array(self.diff))
		sys.exit(0)

if __name__ == "__main__":
		m = MainObject()


