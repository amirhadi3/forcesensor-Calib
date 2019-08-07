
from sensor import sensor
from PyQt5.QtWidgets import (QApplication, QLineEdit, QPushButton, QHBoxLayout, QWidget, QRadioButton, QVBoxLayout, QLabel)
from PyQt5.QtGui import QDoubleValidator
import numpy as np
from pyqtgraph.widgets.MatplotlibWidget import MatplotlibWidget
from random import randint

class  gui(QWidget):
	def __init__(self, sensor, app):
		QWidget.__init__(self)
		rospy.init_node('force_sensor_gui')

		self.rate = 1500
		self.sensor = sensor
		self.transducer_num = 1
		self.num = 0
		self.measurement = None

		self.lrLayout = QHBoxLayout()
		self.layout = QVBoxLayout()
		self.window = QWidget()

		if sensor.num_sensors == 2:
			self.device_num_selector = QRadioButton('Controlling Sensor 1 (click to toggle)')
			self.device_num_selector.toggled.connect(self.device_num_selector_callback)
			self.layout.addWidget(self.device_num_selector)

		self.start_data_transfer_button = QPushButton('Start Data Transfer')
		self.start_data_transfer_button.clicked.connect(self.start_data_transfer_wrapper)
		self.layout.addWidget(self.start_data_transfer_button)

		self.stop_data_transfer_button = QPushButton('Stop Data Transfer')
		self.stop_data_transfer_button.clicked.connect(self.stop_data_transfer_wrapper)
		self.layout.addWidget(self.stop_data_transfer_button)

		self.measure_button = QPushButton('Request One Measurement')
		self.measure_button.clicked.connect(self.measure_wrapper)
		self.layout.addWidget(self.measure_button)

		self.reset_button = QPushButton('Reset Device')
		self.reset_button.clicked.connect(self.reset_wrapper)
		self.layout.addWidget(self.reset_button)

		self.transducer_num_input = QLineEdit()
		self.transducer_num_input.setPlaceholderText('Enter Transducer Number (1-6)')
		self.transducer_num_input.returnPressed.connect(self.update_transducer_num)
		self.layout.addWidget(self.transducer_num_input)

		self.reset_transducer_button = QPushButton('Reset Selected Transducer')
		self.reset_transducer_button.clicked.connect(self.reset_transducer_wrapper)
		self.layout.addWidget(self.reset_transducer_button)

		self.reset_imu_button = QPushButton('Reset IMU')
		self.reset_imu_button.clicked.connect(self.reset_imu_wrapper)
		self.layout.addWidget(self.reset_imu_button)

		self.reset_dac_button = QPushButton('Reset DAC')
		self.reset_dac_button.clicked.connect(self.reset_dac_wrapper)
		self.layout.addWidget(self.reset_dac_button)

		self.sampling_rate_input = QLineEdit()
		self.sampling_rate_input.setValidator(QDoubleValidator(0.1, 1500, 3))
		self.sampling_rate_input.setPlaceholderText('Enter Sampling Rate (Hz) (<1500Hz only)')
		self.sampling_rate_input.returnPressed.connect(self.update_sample_rate)
		self.layout.addWidget(self.sampling_rate_input)

		self.lrLayout.addLayout(self.layout)

		self.pltwidget1 = MatplotlibWidget()
		self.pltwidget1.getFigure().suptitle('Sensor 1 Output',fontsize=16)
		self.diffs1 = self.pltwidget1.getFigure().add_subplot(311)
		self.diffs1.set_title('Differential Signal',fontsize=10)
		self.diffs1.set_ylabel('Volts')
		self.sums1 = self.pltwidget1.getFigure().add_subplot(312)
		self.sums1.set_title('Sum Signal',fontsize=10)
		self.sums1.set_ylabel('Volts')
		self.imu1 = self.pltwidget1.getFigure().add_subplot(313)
		self.imu1.set_title('IMU Data',fontsize=10)
		self.imu1.set_ylabel('Volts')
		self.diff_data1 = np.zeros((1,6))
		self.sum_data1 = np.zeros((1,6))
		self.imu_data1 = np.zeros((1,3))
		self.lrLayout.addWidget(self.pltwidget1)

		self.pltwidget2 = MatplotlibWidget()
		self.pltwidget2.getFigure().suptitle('Sensor 2 Output',fontsize=16)
		self.diffs2 = self.pltwidget2.getFigure().add_subplot(311)
		self.diffs2.set_title('Differential Signal',fontsize=10)
		self.diffs2.set_ylabel('Volts')
		self.sums2 = self.pltwidget2.getFigure().add_subplot(312)
		self.sums2.set_title('Sum Signal',fontsize=10)
		self.sums2.set_ylabel('Volts')
		self.imu2 = self.pltwidget2.getFigure().add_subplot(313)
		self.imu2.set_title('IMU Data',fontsize=10)
		self.imu2.set_ylabel('Volts')
		self.diff_data2 = np.zeros((1,6))
		self.sum_data2 = np.zeros((1,6))
		self.imu_data2 = np.zeros((1,3))
		self.lrLayout.addWidget(self.pltwidget2)

		self.colors = ['#FF0000', '#FFA500', '#FFBF00', '#00FF00', '#FFFFFF'] # red, orange, yellow, green, white
		self.acc1 = 4
		self.acc2 = 4

		sub = rospy.Subscriber('continuous_data', SensorOutput, self.data_subscriber)

		self.window.setLayout(self.lrLayout)
		self.window.show()

	def update_sample_rate(self):
		self.rate = float(self.sampling_rate_input.text())
	def start_data_transfer_wrapper(self):
		self.sensor.start_data_transfer(self.num, self.rate)
	def stop_data_transfer_wrapper(self):
		self.sensor.stop_data_transfer(self.num)
	def measure_wrapper(self):
		self.measurement = self.sensor.measure(self.num)
	def reset_wrapper(self):
		self.sensor.reset_device(self.num)
	def reset_transducer_wrapper(self):
		self.sensor.reset_transducer(self.transducer_num, self.num)
	def reset_imu_wrapper(self):
		self.sensor.reset_imu(self.num)
	def reset_dac_wrapper(self):
		self.sensor.reset_dac(self.num)
	def update_transducer_num(self):
		self.transducer_num = int(self.transducer_num_input.text())
		self.reset_transducer_button.setText('Reset Transducer ' + str(self.transducer_num))
		#self.reset_transducer_wrapper()
	def device_num_selector_callback(self):
		if self.device_num_selector.isChecked():
			self.num = 1
			self.device_num_selector.setText('Controlling Sensor 2')
		else:
			self.num = 0
			self.device_num_selector.setText('Controlling Sensor 1')
	def data_subscriber(self, msg):
		if msg.sensor_num == 0:
			self.diff_data1.append(msg.differentials)
			if len(self.diff_data1) > 10000:
				self.diff_data1 = self.diff_data1[-10000:]
			self.sum_data1.append(msg.sum)
			if len(self.sum_data1) > 10000:
				self.sum_data1 = self.sum_data1[-10000:]
			self.imu_data1.append(msg.imu)
			if len(self.imu_data1) > 10000:
				self.imu_data1 = self.imu_data1[-10000:]
			self.acc1 = msg.accuracy

		elif msg.sensor_num == 1:
			self.diff_data2.append(msg.differentials)
			if len(self.diff_data2) > 10000:
				self.diff_data2 = self.diff_data2[-10000:]
			self.sum_data2.append(msg.sum)
			if len(self.sum_data2) > 10000:
				self.sum_data2 = self.sum_data2[-10000:]
			self.imu_data2.append(msg.imu)
			if len(self.imu_data2) > 10000:
				self.imu_data2 = self.imu_data2[-10000:]
			self.acc2 = msg.accuracy
		self.plotData()
	def plotData(self):
		self.diffs1.plot(diff_data1,color=self.colors[self.acc1])
		self.diffs2.plot(diff_data2,color=self.colors[self.acc2])

		self.sum1.plot(diff_sum1,color=self.colors[self.acc1])
		self.sum2.plot(diff_sum2,color=self.colors[self.acc2])

		self.imu1.plot(diff_imu1,color=self.colors[self.acc1])
		self.imu2.plot(diff_imu2,color=self.colors[self.acc2])

		self.pltwidget1.draw()
		self.pltwidget2.draw()

if __name__ == "__main__":
	
	sensor = Sensor(num_sensors=2)
	app = QApplication([])
	GUI = gui(sensor, app)

	app.exec_()
