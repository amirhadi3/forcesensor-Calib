
from sensor import Sensor
from PyQt5.QtWidgets import (QApplication, QLineEdit, QPushButton, QHBoxLayout, QWidget, QRadioButton, QVBoxLayout, QLabel, QDialog, QMainWindow, QTabWidget, QFileDialog, QCheckBox, QGridLayout, QComboBox, QListWidget)
from PyQt5.QtGui import QDoubleValidator
from PyQt5 import QtGui, QtCore
import numpy as np
from math import atan2, asin
import time
import sys
from pyqtgraph.widgets.MatplotlibWidget import MatplotlibWidget
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from datetime import datetime as date
import xml.etree.cElementTree as xml
import sensor_output as out
import signal

FORCE = 0
TORQUE = 1
DIFF = 2
SUM = 3
IMU = 4
TEMP = 5

XYZ = True
ABC = False

GAIN = 0
SPS = 1
AUTOCAL = 2
INBUF = 3
POSCHAN = 4
NEGCHAN = 5
LEDCURR = 6


class App(QMainWindow):
	def __init__(self,quick=False):
		super(QMainWindow, self).__init__()

		self.title = "Optical Force Sensor Control Panel"
		self.left = 0
		self.top = 0
		self.width = 2000
		self.height = 1000
		self.setWindowTitle(self.title)
		self.setGeometry(self.left, self.top, self.width, self.height)

		self.control_widget = ControlWidget(self,quick)
		self.setCentralWidget(self.control_widget)

		self.show()

class LoadScreen(QMainWindow):
	def __init__(self):
		super(QMainWindow,self).__init__()
		self.setWindowTitle("Loading Application...")
		self.setGeometry(700,500,500,50)
		self.setCentralWidget(QtGui.QWidget(self))
		self.layout = QVBoxLayout()
		self.centralWidget().setLayout(self.layout)
		self.loading = QLabel('Loading...')
		self.layout.addWidget(self.loading)
		self.show()

class  ControlWidget(QWidget):
	update = QtCore.pyqtSignal(int,list,bool)

	def __init__(self, parent, quick=False):
		super(QWidget, self).__init__(parent)
		self.layout = QVBoxLayout(self)

		self.loadScreen = LoadScreen()
		
		self.calMatrix = np.eye(6)
		self.sensor = Sensor(self.calMatrix,quick=quick)
		self.sensor.purge()

		self.calibrateWindow = CalibrationWindow(self.sensor)

		#Some parameters
		self.rate = 1500
		self.plotCount = 0
		self.fps = 10
		self.plotInterval = self.rate / self.fps
		self.num = 0
		self.log = False
		self.path = None
		self.previously_selected = []
		self.imuValLabel = 'r'
		self.imu_y_label = 'Unit Quaternion'
		self. timestamp = self.makeTimestamp() 
		self.imu_rate = 20 #Hz
		self.ignoreEvents = False
		if not quick:
			self.sensor.config_imu(8,self.imu_rate)

		self.adsState = np.array([[0,15,1,0,3,2,0],[0,15,1,0,3,2,0],[0,15,1,0,3,2,0],[0,15,1,0,3,2,0],[0,15,1,0,3,2,0],[0,15,1,0,3,2,0]])
		self.colors_named = ["color:red", "color: blue", "color: green", "color: magenta", "color: brown", "color: orange"]
		lengthList = [3,3,6,6,3]
		self.oldState = [[0,0,0],[0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]]

		textbf = QtGui.QFont()
		textbf.setBold(True)

		#Initialize tabs
		self.tabs = QTabWidget()
		self.baseTab = QWidget()
		self.funcTab = QWidget()
		self.imuTab = QWidget()

		self.tabs.resize(parent.width, parent.height)

		self.tabs.addTab(self.baseTab, "Home")
		self.tabs.addTab(self.funcTab, "Utilities")
		self.tabs.addTab(self.imuTab, "IMU")

		#######################################      Base Tab    #######################################
		self.baseTab.layout = QGridLayout(self.baseTab)
		self.baseTab.layout.setSpacing(10)
		self.contiMode = False

		#Start/Stop button
		self.start_button = QPushButton('Start')
		self.start_button.clicked.connect(self.start_data_transfer_wrapper)
		self.baseTab.layout.addWidget(self.start_button,0,0,1,1)

		self.stop_button = QPushButton('Stop')
		self.stop_button.clicked.connect(self.stop_data_transfer_wrapper)
		self.baseTab.layout.addWidget(self.stop_button,0,1,1,1)

		#Data logging checkbox
		self.set_data_log_checkbox = QRadioButton('Log Data')
		self.set_data_log_checkbox.toggled.connect(self.log_data)
		self.baseTab.layout.addWidget(self.set_data_log_checkbox,1,0,1,1)

		self.log_limit_input = QLineEdit()
		self.log_limit_input.setPlaceholderText('Measure Time (sec) (Currently: inf)')
		self.log_limit_input.returnPressed.connect(self.update_log_limit)
		self.baseTab.layout.addWidget(self.log_limit_input,1,1,1,1)	

		#Edit Sampling Rate
		self.sampling_rate_label = QLabel('Enter Sampling Rate (Hz) (<3000Hz only)')
		self.sampling_rate_input = QLineEdit()
		self.sampling_rate_input.setValidator(QDoubleValidator(0.1, 3000, 3))
		self.sampling_rate_input.setPlaceholderText('Default = 1500Hz')
		self.sampling_rate_input.returnPressed.connect(self.update_sample_rate)
		self.baseTab.layout.addWidget(self.sampling_rate_input,2,1,1,1)	
		self.baseTab.layout.addWidget(self.sampling_rate_label,2,0,1,1)

		#Poll
		self.measure_button = QPushButton('Poll One Measurement')
		self.measure_button.clicked.connect(self.measure_wrapper)
		self.baseTab.layout.addWidget(self.measure_button,3,0,1,2)	

		#Get Calibration Matrix
		self.cal_matrix_button = QPushButton('Calibration Matrix')
		self.cal_matrix_button.clicked.connect(self.get_cal_matrix)
		self.baseTab.layout.addWidget(self.cal_matrix_button,4,0,1,2)

		self.reset_button = QPushButton('Reset Device')
		self.reset_button.clicked.connect(self.reset_wrapper)
		self.baseTab.layout.addWidget(self.reset_button,5,0,1,2)

		#Temperature display
		self.temp_display = QLabel('Temperature: (Start data to view)')
		self.baseTab.layout.addWidget(self.temp_display,6,0,1,2)

		self.saturated_label = QLabel('No channels saturated')
		self.saturated_label.setStyleSheet('color: green')
		self.baseTab.layout.addWidget(self.saturated_label,7,0,1,2)

		self.f_val = QLabel('Fx:\t\t(start or poll to view)\n\nFy:\t\t(start or poll to view)\n\nFz:\t\t(start or poll to view)\n\nTx:\t\t(start or poll to view)\n\nTy:\t\t(start or poll to view)\n\nTz:\t\t(start or poll to view)')
		self.baseTab.layout.addWidget(self.f_val, 8,0,4,2)

		self.status= QLabel('Status: ready')
		self.baseTab.layout.addWidget(self.status,12,0,1,2)

		#Edit Frame Length
		self.frame_length_input = []
		self.frame_length_input.append(QLineEdit())
		self.frame_length_input[0].setPlaceholderText('Enter Frame Length (Number of Seconds to Plot)')
		self.frame_length_input[0].returnPressed.connect(self.update_frame_length)
		self.baseTab.layout.addWidget(self.frame_length_input[0],0,9,1,1)	

		self.frame_length_input.append(QLineEdit())
		self.frame_length_input[1].setPlaceholderText('Enter Frame Length (Number of Seconds to Plot)')
		self.frame_length_input[1].returnPressed.connect(self.update_frame_length)
		self.baseTab.layout.addWidget(self.frame_length_input[1],12,9,1,1)	

		self.show_orientation_button = QCheckBox('Show Live Orientation Plot',self)
		self.show_orientation_button.setChecked(True)
		self.show_orientation_button.stateChanged.connect(self.show_orientation)
		self.baseTab.layout.addWidget(self.show_orientation_button,19,0,1,2)

		#######################################     Func Tab     #######################################
		self.funcTab.layout = QGridLayout(self.funcTab)
		self.funcTab.layout.setSpacing(10)
		
		#Start/Stop button
		self.start_button2 = QPushButton('Start')
		self.start_button2.clicked.connect(self.start_data_transfer_wrapper)
		self.funcTab.layout.addWidget(self.start_button2,0,0,1,1)

		self.stop_button2 = QPushButton('Stop')
		self.stop_button2.clicked.connect(self.stop_data_transfer_wrapper)
		self.funcTab.layout.addWidget(self.stop_button2,0,1,1,1)

		self.channel_label = QLabel('Select Channel:')
		self.funcTab.layout.addWidget(self.channel_label,1,0,1,1)

		self.channel_num_input = QComboBox()
		self.channel_num_input.addItems(['A','B','C','D','E','F','All'])
		self.channel_num_input.activated.connect(self.update_channel_num)
		self.funcTab.layout.addWidget(self.channel_num_input,1,1,1,1)

		self.misc_label = QLabel('ADS Control:')
		self.funcTab.layout.addWidget(self.misc_label,2,0,1,2)
		self.misc_label.setFont(textbf)

		self.reset_ads_button = QPushButton('On / Reset')
		self.reset_ads_button.clicked.connect(self.reset_ads_wrapper)
		self.funcTab.layout.addWidget(self.reset_ads_button,3,0,1,1)

		self.reset_ads_button = QPushButton('Off')
		self.reset_ads_button.clicked.connect(self.deactivate_ads_wrapper)
		self.funcTab.layout.addWidget(self.reset_ads_button,3,1,1,1)

		self.gain_label = QLabel('Gain:')
		self.funcTab.layout.addWidget(self.gain_label,4,0,1,1)

		self.gain_input = QComboBox()
		self.gain_input.addItems(['1','2','4','8','16','32','64'])
		self.gain_input.activated.connect(self.update_gain)
		self.funcTab.layout.addWidget(self.gain_input,4,1,1,1)

		self.rate_label = QLabel('Sampling Rate:')
		self.funcTab.layout.addWidget(self.rate_label,5,0,1,1)

		self.sps_input = QComboBox()
		self.sps_input.addItems(['2.5', '5', '10', '15', '25', '30', '50', '60', '100', '500', '1e3', '2e3', '3.75e3', '7.5e3', '15e3', '30e3'])
		self.sps_input.setCurrentIndex(self.adsState[0,SPS])
		self.sps_input.activated.connect(self.update_sps)
		self.funcTab.layout.addWidget(self.sps_input,5,1,1,1)

		self.autocal_button = QCheckBox('Auto Calibration',self)
		self.autocal_button.setChecked(True)
		self.autocal_button.stateChanged.connect(self.autocal)
		self.funcTab.layout.addWidget(self.autocal_button,6,1,1,2)

		self.buffer_button = QCheckBox('Input Buffer',self)
		self.buffer_button.stateChanged.connect(self.autocal)
		self.funcTab.layout.addWidget(self.buffer_button,7,1,1,2)

		self.pos_label = QLabel('POS_Channel:')
		self.funcTab.layout.addWidget(self.pos_label,8,0,1,1)

		self.pos_input = QComboBox()
		self.pos_input.addItems(['0','1','2','3'])
		self.pos_input.setCurrentIndex(3)
		self.pos_input.activated.connect(self.update_pos_chan)
		self.funcTab.layout.addWidget(self.pos_input,8,1,1,1)

		self.neg_label = QLabel('NEG_Channel:')
		self.funcTab.layout.addWidget(self.neg_label,9,0,1,1)

		self.neg_input = QComboBox()
		self.neg_input.addItems(['0','1','2','3'])
		self.neg_input.setCurrentIndex(2)
		self.neg_input.activated.connect(self.update_neg_chan)
		self.funcTab.layout.addWidget(self.neg_input,9,1,1,1)

		self.calibrate_button = QPushButton('Run Self-Calibration')
		self.calibrate_button.clicked.connect(self.self_calibrate)
		self.funcTab.layout.addWidget(self.calibrate_button, 10,0,1,2)

		self.ofc_button = QPushButton('Request OFC and FSC Registers')
		self.ofc_button.clicked.connect(self.request_ofc_fsc)
		self.funcTab.layout.addWidget(self.ofc_button, 11,0,1,2)

		self.saturated_label2 = QLabel('No channels saturated')
		self.saturated_label2.setStyleSheet('color: green')
		self.funcTab.layout.addWidget(self.saturated_label2,12,0,1,2)

		self.status2= QLabel('Status: ready')
		self.funcTab.layout.addWidget(self.status2,13,0,1,2)

		self.led_label = QLabel('LED Control:')
		self.funcTab.layout.addWidget(self.led_label,14,0,1,2)
		self.led_label.setFont(textbf)

		self.misc_label = QLabel('Select LEDs to Turn On')
		self.funcTab.layout.addWidget(self.misc_label,15,0,1,2)

		self.led_select = QListWidget()
		self.led_select.addItems(['LED A', 'LED B', 'LED C', 'LED D', 'LED E', 'LED F'])
		self.led_select.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)
		self.led_select.itemClicked.connect(self.led_control)
		self.funcTab.layout.addWidget(self.led_select,16,0,1,2)

		self.led_set_label = QLabel('Set LED A Current')
		self.funcTab.layout.addWidget(self.led_set_label, 17,0,1,2)

		self.led_current_set = QLineEdit()
		self.led_current_set.setPlaceholderText('Currently: 0')
		self.led_current_set.returnPressed.connect(self.set_led_current)
		self.funcTab.layout.addWidget(self.led_current_set, 18,0,1,2)

		self.ma_label = QLabel('mA')
		self.funcTab.layout.addWidget(self.ma_label,18,1,1,1)

		self.reset_dac_button = QPushButton('Reset DAC')
		self.reset_dac_button.clicked.connect(self.reset_dac_wrapper)
		self.funcTab.layout.addWidget(self.reset_dac_button,19,0,1,2)

		self.poll_button_func = QPushButton('Poll One Measurement')
		self.poll_button_func.clicked.connect(self.measure_wrapper)
		self.funcTab.layout.addWidget(self.poll_button_func,20,0,1,2)

		self.frame_length_input.append(QLineEdit())
		self.frame_length_input[2].setPlaceholderText('Enter Frame Length (Number of Seconds to Plot)')
		self.frame_length_input[2].returnPressed.connect(self.update_frame_length)
		self.funcTab.layout.addWidget(self.frame_length_input[2],0,9,1,1)	

		self.frame_length_input.append(QLineEdit())
		self.frame_length_input[3].setPlaceholderText('Enter Frame Length (Number of Seconds to Plot)')
		self.frame_length_input[3].returnPressed.connect(self.update_frame_length)
		self.funcTab.layout.addWidget(self.frame_length_input[3],12,9,1,1)

		#Normalize all the row widths
		for i in range(1,20):
			self.funcTab.layout.setRowStretch(i,1)

		#######################################      IMU Tab      #######################################
		self.imuTab.layout = QGridLayout(self.imuTab)

		#Start/Stop button
		self.start_button3 = QPushButton('Start')
		self.start_button3.clicked.connect(self.start_data_transfer_wrapper)
		self.imuTab.layout.addWidget(self.start_button3,0,0,1,1)
		self.stop_button3 = QPushButton('Stop')
		self.stop_button3.clicked.connect(self.stop_data_transfer_wrapper)
		self.imuTab.layout.addWidget(self.stop_button3,0,1,1,1)

		self.reset_imu_button = QPushButton('Reset IMU')
		self.reset_imu_button.clicked.connect(self.reset_imu_wrapper)
		self.imuTab.layout.addWidget(self.reset_imu_button,1,0,1,2)

		self.mode_label = QLabel('Select IMU Output Mode: ')
		self.imuTab.layout.addWidget(self.mode_label, 2,0,1,1)

		self.mode_input = QComboBox()
		self.mode_input.addItems(['Game Rotation Vector','Accelerometer','Gyroscope','Linear Acceleration','Rotation Vector'])
		self.mode_input.activated.connect(self.update_mode)
		self.imuTab.layout.addWidget(self.mode_input,2,1,1,2)

		#Edit Sampling Rate
		self.sample_rate_label = QLabel('Enter IMU Sampling Rate (Hz): ')
		self.imuTab.layout.addWidget(self.sample_rate_label, 3,0,1,1)

		self.sampling_rate_input2 = QLineEdit()
		self.sampling_rate_input2.setPlaceholderText('Default: 20Hz (<1kHz only)')
		self.sampling_rate_input2.returnPressed.connect(self.update_sample_rate_imu)
		self.imuTab.layout.addWidget(self.sampling_rate_input2,3,1,1,2)	

		#Poll
		self.poll_button = QPushButton('Poll One Measurement')
		self.poll_button.clicked.connect(self.measure_wrapper)
		self.imuTab.layout.addWidget(self.poll_button,4,0,1,2)	

		self.calibrate_imu_button = QPushButton('Calibrate IMU')
		self.calibrate_imu_button.clicked.connect(self.calibrate_imu)
		self.imuTab.layout.addWidget(self.calibrate_imu_button,5,0,1,2)

		self.status3= QLabel('Status: ready')
		self.imuTab.layout.addWidget(self.status3,6,0,1,2)

		self.x_label = QLabel('q_i:\n\n(Start to View)')
		self.imuTab.layout.addWidget(self.x_label, 7,0,2,1)

		self.y_label = QLabel('q_j:\n\n(Start to View)')
		self.imuTab.layout.addWidget(self.y_label, 7,1,2,1)

		self.z_label = QLabel('q_k:\n\n(Start to View)')
		self.imuTab.layout.addWidget(self.z_label, 7,2,2,1)

		self.read_button = QPushButton('Read Input Buffer')
		self.read_button.clicked.connect(self.read_stuff)
		self.imuTab.layout.addWidget(self.read_button, 9, 0, 1, 1)

		self.frame_length_input.append(QLineEdit())
		self.frame_length_input[4].setPlaceholderText('Enter Frame Length (Number of Seconds to Plot)')
		self.frame_length_input[4].returnPressed.connect(self.update_frame_length)
		self.imuTab.layout.addWidget(self.frame_length_input[4],0,8,1,1)	

		self.frame_length_input.append(QLineEdit())
		self.frame_length_input[5].setPlaceholderText('Enter Frame Length (Number of Seconds to Plot)')
		self.frame_length_input[5].returnPressed.connect(self.update_frame_length)
		self.imuTab.layout.addWidget(self.frame_length_input[5],12,8,1,1)

		# Create axis options buttons
		self.axisButtons = []
		self.axisLabels = []
		labels = ['1 ---', '2 ---', '3 ---', '4 ---', '5 ---', '6 ---']
		labelsxyz = ['x ---', 'y ---', 'z ---']

		for i in range(5):
			self.axisLabels.append(QLabel('Select Axes: '))
			self.axisLabels[i].setFont(textbf)

			button_list = []
			for j in range(lengthList[i]):
				if i == 2 or i == 3:
					button_list.append(QCheckBox(labels[j]))
				else:
					button_list.append(QCheckBox(labelsxyz[j]))
				button_list[j].setStyleSheet(self.colors_named[j])
				button_list[j].stateChanged.connect(self.update_plot_curves)
			self.axisButtons.append(button_list)

		# Add buttons to tabs
		for i in range(6):
			self.funcTab.layout.addWidget(self.axisButtons[DIFF][i], 0,3+i,1,1)
			self.funcTab.layout.addWidget(self.axisButtons[SUM][i],12,3+i,1,1)
		for i in range(3):
			self.imuTab.layout.addWidget(self.axisButtons[IMU][i], 0,5+i,1,1)
			self.baseTab.layout.addWidget(self.axisButtons[FORCE][i],0,6+i,1,1)
			self.baseTab.layout.addWidget(self.axisButtons[TORQUE][i],12,6+i,1,1)

		self.baseTab.layout.addWidget(self.axisLabels[FORCE],0,5,1,1)
		self.baseTab.layout.addWidget(self.axisLabels[TORQUE],12,5,1,1)
		self.funcTab.layout.addWidget(self.axisLabels[DIFF],0,2,1,1)
		self.funcTab.layout.addWidget(self.axisLabels[SUM],12,2,1,1)
		self.imuTab.layout.addWidget(self.axisLabels[IMU],0,4,1,1)

		#Finish Setting up overall layout
		self.loadScreen.hide()
		self.layout.addWidget(self.tabs)
		self.setLayout(self.layout)

		self.plotThread = PlotThread(self, self.sensor, self.log, self.rate, self.tabs, self.baseTab, self.funcTab, self.imuTab)
		self.plotThread.new_data.connect(self.data_subscriber)
		self.plotThread.finished1.connect(self.onFinished)

		#######################################     Wrappers      #######################################

	def calibrate_imu(self):
		self.calibrateWindow.show()

	def set_status(self, msg):
		if msg == '':
			msg = 'No response received'
		self.status.setText(msg)
		self.status2.setText(msg)
		self.status3.setText(msg)

	def update_sample_rate(self):
		self.rate = float(self.sampling_rate_input.text())
		self.sampling_rate_input.clear()
		self.sampling_rate_input.setPlaceholderText('Currently: ' + str(self.rate))

	def update_sample_rate_imu(self):
		self.imu_rate = float(self.sampling_rate_input2.text())
		self.update_mode()
	
	def stop_data_transfer_wrapper(self):
		if self.contiMode:
			self.set_status(self.sensor.stop_data_transmission())
			self.contiMode = False

			self.plotThread.read = False
			self.plotThread.quit()
			time.sleep(0.5)
			
			if self.plotThread.isRunning():
				self.plotThread.quit()
				time.sleep(0.5)
				if self.plotThread.isRunning():
					print("Error thread still running")
					self.set_status('Could not stop data transfer')
			else:
				self.set_status('Stopped data transfer')
		
	def start_data_transfer_wrapper(self):
		if not self.contiMode:
			self.diff_data = []
			self.sum_data = []
			self.ft = []
			self.imu_data = []

			self.timestamp = self.makeTimestamp()
			self.contiMode = True
			self.sensor.start_data_transmission(data_rate=self.rate)
			self.set_status('Started data transfer')
			time.sleep(0.5)

			if self.plotThread != None:
				del self.plotThread
			self.plotThread = PlotThread(self, self.sensor, self.log, self.rate, self.tabs, self.baseTab, self.funcTab, self.imuTab)
			self.plotThread.new_data.connect(self.data_subscriber)
			self.plotThread.finished1.connect(self.onFinished)
			self.plotThread.start()
	
	def request_ofc_fsc(self):
		ofc, fsc = self.sensor.ads_report_registers(self.num)
		self.set_status('OFC: ' + str(ofc) + '\nFSC: ' + str(fsc))

	def update_frame_length(self):
		count = 0
		while count < len(self.frame_length_input) and self.frame_length_input[count].text() == '':
			count += 1
		
		if count < len(self.frame_length_input):
			s = self.frame_length_input[count].text()
			self.update.emit(1,[count, int(s)],True)

	def read_stuff(self):
		string = ''
		d = self.sensor.readBytes(1)
		if d != []:
				s = "%x" % d[0]
				if len(s) < 2:
					s = '0' + s
				string += s
		
		count = 0
		empty = 0
		while count < 100000 and empty < 10:
			if d == []:
				empty += 1
			elif empty != 0:
				empty = 0
			d = self.sensor.readBytes(1)
			if d != []:
				s = "%x" % d[0]
				if len(s) < 2:
					s = '0' + s
				string += s
			else:
				string += ''
			count += 1
		self.set_status(string.decode('hex'))
		if count == 10000:
			print('Failed to empty buffer. Read 10,000 bytes')

	@QtCore.pyqtSlot(list,list,float,list)
	def data_subscriber(self, imu, force, temp, saturated):
		self.f_val.setText('Fx:\t  '+str(round(force[0],4))+'N\n\nFy:\t  '+str(round(force[1],4))+'N\n\nFz:\t  '+str(round(force[2],4))+'N\n\nTx:\t  '+str(round(force[3],4))+'N.mm\n\nTy:\t  '+str(round(force[4],4))+'N.mm\n\nTz:\t  '+str(round(force[5],4))+'N.mm')
		self.temp_display.setText('Temperature: ' + str(round(temp,4)) + 'C')
		if self.tabs.currentIndex() == 2:
			self.x_label.setText(self.imuValLabel + '_x\n\n' + str(round(imu[0],4)))
			self.y_label.setText(self.imuValLabel + '_y\n\n' + str(round(imu[1],4)))
			self.z_label.setText(self.imuValLabel + '_z\n\n' + str(round(imu[2],4)))
		if sum(saturated) > 0:
			self.saturated_label.setText('Caution, saturated in channels ' + str(saturated))
			self.saturated_label.setStyleSheet('color: red')
			self.saturated_label2.setText('Caution, saturated in channels ' + str(saturated))
			self.saturated_label2.setStyleSheet('color: red')
		else:
			self.saturated_label.setText('No channels saturated')
			self.saturated_label.setStyleSheet('color: green')
			self.saturated_label2.setText('No channels saturated')
			self.saturated_label2.setStyleSheet('color: green')

	def measure_wrapper(self):
		sample = self.sensor.poll()

		if sample != None:
			self.f_val.setText('Fx:\t  '+str(round(sample.wrench[0],4))+'N\n\nFy:\t  '+str(round(sample.wrench[1],4))+'N\n\nFz:\t  '+str(round(sample.wrench[2],4))+'N\n\nTx:\t  '+str(round(sample.wrench[3],4))+'N.mm\n\nTy:\t  '+str(round(sample.wrench[4],4))+'N.mm\n\nTz:\t  '+str(round(sample.wrench[5],4))+'N.mm')
			self.temp_display.setText('Temperature: ' + str(sample.temperature) + 'C')
			self.x_label.setText(self.imuValLabel + '_x\n\n' + str(sample.imu[0]))
			self.y_label.setText(self.imuValLabel + '_y\n\n' + str(sample.imu[1]))
			self.z_label.setText(self.imuValLabel + '_z\n\n' + str(sample.imu[2]))

			if sample.isSaturated():
				self.saturated_label.setText('Caution, saturated in channels ' + str(sample.saturated))
				self.saturated_label.setStyleSheet('color: red')
				self.saturated_label2.setText('Caution, saturated in channels ' + str(sample.saturated))
				self.saturated_label2.setStyleSheet('color: red')
			else:
				self.saturated_label.setText('No channels saturated')
				self.saturated_label.setStyleSheet('color: green')
				self.saturated_label2.setText('No channels saturated')
				self.saturated_label2.setStyleSheet('color: green')

			self.update.emit(3, [sample], True)
		
		else:
			self.f_val.setText('Fx:\t\t(failed)\n\nFy:\t\t(failed)\n\nFz:\t\t(failed)\n\nTx:\t\t(failed)\n\nTy:\t\t(failed)\n\nTz:\t\t(failed)')
			self.temp_display.setText('Temperature: (failed) deg Celsius')
			self.x_label.setText(self.imuValLabel + '_x\n\n(failed)')
			self.y_label.setText(self.imuValLabel + '_y\n\n(failed)')
			self.z_label.setText(self.imuValLabel + '_z\n\n(failed)')


	def reset_wrapper(self):
		self.set_status(self.sensor.reset_device())
	def self_calibrate(self):
		self.set_status(self.sensor.ads_self_calibrate(self.num))
	def reset_ads_wrapper(self):
		self.set_status(self.sensor.reset_ads(self.num))
	def deactivate_ads_wrapper(self):
		self.set_status(self.sensor.deactivate_ads(self.num))
	def reset_imu_wrapper(self):
		self.set_status(self.sensor.reset_imu())
	
	def reset_dac_wrapper(self):
		self.set_status(self.sensor.reset_dac())
		self.led_current_set.clear()
		self.led_current_set.setPlaceholderText('Currently: 0.0')

	def update_channel_num(self):
		self.num = self.channel_num_input.currentIndex()
		if self.num == 6:
			self.num=-1
		letters = ['A','B','C','D','E','F','All']
		self.led_set_label.setText('Set LED ' + letters[self.num] + ' current')
		self.update_utilities_page(self.num)

	def update_utilities_page(self, adsNum):
		self.gain_input.setCurrentIndex(self.adsState[adsNum,GAIN])
		self.sps_input.setCurrentIndex(self.adsState[adsNum,SPS])
		self.pos_input.setCurrentIndex(self.adsState[adsNum,POSCHAN])
		self.neg_input.setCurrentIndex(self.adsState[adsNum,NEGCHAN])
		self.autocal_button.setChecked(True if self.adsState[adsNum,AUTOCAL]==1 else False)
		self.buffer_button.setChecked(True if self.adsState[adsNum,INBUF]==1 else False)
		self.led_current_set.clear()
		self.led_current_set.setPlaceholderText('Currently: ' + str(self.adsState[self.num,LEDCURR]))

	@QtCore.pyqtSlot(list, list, list)
	def onFinished(self,force,imu, temp):
		
		if self.plotThread.isRunning():
			self.plotThread.quit()
			time.sleep(0.5)
			if self.plotThread.isRunning():
				print("Error. Thread still running.")

		if self.log:
			self.save_data(force, imu, temp)

	def save_data(self, force, imu, temp):
		with open(self.path + '/diff_' + self.timestamp + '.csv','ab') as file:
			np.savetxt(file,np.array(force),delimiter=',')
		with open(self.path + '/sum_' + self.timestamp + '.csv','ab') as file:
			np.savetxt(file,np.array(imu),delimiter=',')
		with open(self.path + '/temp_' + self.timestamp + '.csv','ab') as file:
			np.savetxt(file,np.array(temp),delimiter=',')

	def makeTimestamp(self):
		d = date.now()
		return str(d)[:10] + '_' + str(d)[11:13] + '-' +  str(d)[14:16] + '-' + str(d)[17:19]

	def update_plot_curves(self):
		idx = self.tabs.currentIndex()
		ranges = [[0,1],[2,3],[4]]
		for j in ranges[idx]:
			for i in range(len(self.axisButtons[j])):
				if self.axisButtons[j][i].isChecked() and self.oldState[j][i] == 0:
					self.update.emit(0,[j,i],True)
					self.oldState[j][i] = 1
				elif not self.axisButtons[j][i].isChecked() and self.oldState[j][i] == 1:
					self.update.emit(0,[j,i],False)
					self.oldState[j][i] = 0

	def update_log_limit(self):
		secs = int(self.log_limit_input.text())
		if secs == -1:
			self.update.emit(6,[0],False)
			self.log_limit_input.clear()
			self.log_limit_input.setPlaceholderText('Measure Time (sec) (Currently: inf)')
		else:
			lim = int(secs*self.rate)
			self.log_limit_input.clear()
			self.log_limit_input.setPlaceholderText('Measure Time (sec) (Currently: ' + str(secs) + ' )')
			self.update.emit(6,[lim],True)

	def log_data(self):
		if self.log == False:
			self.log = True
			self.update.emit(4,[],True)
			if self.path == None:
				dirname = str(QFileDialog.getExistingDirectory(self,"Select Directory in which to Save Data"))
				self.path = dirname
		else:
			self.log = False
			self.update.emit(4,[],False)

	def get_cal_matrix(self):
			"""
			Read the .cal file into a calibration matrix
			"""
			filename, _ = QFileDialog.getOpenFileName(self, "Choose Calibration Matrix File","","Calibration Files (*.cal)")
			if filename != '':
				root = xml.parse(str(filename)).getroot()
				axes = root.findall('Calibration/Axis')
				self.calMatrix = np.array([[float(axes[i].attrib['values'].split()[j]) for j in range(6)] for i in range(6)])
			else:
				self.calMatrix = np.eye(6)
			self.sensor.calMatrix = self.calMatrix

	def led_control(self):
		sel = self.led_select.selectedItems()
		selected = [self.led_select.indexFromItem(sel[i]).row() for i in range(len(sel))]
		self.set_status('Turning on LEDs: ' + str(selected))
		for i in range(6):
			if i not in selected and i in self.previously_selected:
				self.sensor.turn_off_led(i)
				self.adsState[i,LEDCURR] = 0
			elif i in selected and i not in self.previously_selected:
				self.sensor.turn_on_led(i)
				self.adsState[i,LEDCURR] = 20
		self.previously_selected = selected

	def set_led_current(self):
		curr = float(self.led_current_set.text())
		self.led_current_set.clear()
		self.led_current_set.setPlaceholderText('Currently: ' + str(curr))
		self.set_status(self.sensor.config_dac(self.num, curr))
		self.set_ads_state(LEDCURR, curr)
		
	def update_gain(self):
		if not self.ignoreEvents:
			gain = int(self.gain_input.currentText())
			self.set_status(self.sensor.set_pga_gain(self.num, gain))
			if self.num != -1:
				self.adsState[self.num,GAIN] = self.gain_input.currentIndex()
				print('Set Gain of ADS ' + str(self.num) + ' to ' + str(gain))
			else:
				self.adsState[:,GAIN] = self.gain_input.currentIndex()
				print("Set Gain of all ADS's to " + str(gain))

	def update_sps(self):
		if not self.ignoreEvents:
			sps = float(self.sps_input.currentText())
			self.set_status(self.sensor.set_ads_drate(self.num, sps))
			self.set_ads_state(SPS, sps)

	def autocal(self):
		if not self.ignoreEvents:
			if self.autocal_button.isChecked():
				ac = 1
				self.set_ads_state(2,1)
			else:
				ac = 0
				self.set_ads_state(2,0)
			if self.buffer_button.isChecked():
				ib = 1
				self.set_ads_state(3,1)
			else:
				ib = 0
				self.set_ads_state(3,0)

			self.set_status(self.sensor.set_ads_registers(self.num,ac,ib))

	def update_pos_chan(self):
		if not self.ignoreEvents:
			self.set_status(self.sensor.set_ads_channel(self.num,int(self.pos_input.currentText()),True))
			self.set_ads_state(4, int(self.pos_input.currentText()))

	def update_neg_chan(self):
		if not self.ignoreEvents:
			self.set_status(self.sensor.set_ads_channel(self.num,int(self.neg_input.currentText()),False))
			self.set_ads_state(5, int(self.neg_input.currentText()))

	def set_ads_state(self, stateNum, state):
		if self.num != -1:
			self.adsState[self.num,stateNum] = state
		else:
			self.adsState[:,stateNum] = state

	def show_orientation(self):
		state = self.show_orientation_button.isChecked()
		if self.log:
			if state:
				self.show_orientation_button.setChecked(False)
			else:
				self.show_orientation_button.setChecked(False)
				self.update.emit(5, [], False)
		else:
			self.update.emit(5, [], state)

	def update_mode(self):
		if self.imu_rate == 0:
			delay = 0
		else:
			delay = int(1.0/self.imu_rate*1000)
		name = self.mode_input.currentText()
		self.set_status(self.sensor.config_imu(name,delay))

		if name == 'Accelerometer':
			self.imuValLabel = 'a'
			self.imu_y_label = 'Acceleration (m/s^2)'
			self.update.emit(2, ['Acceleration (m/s^2)'], XYZ)
			self.axisButtons[IMU][0].setText('X')
			self.axisButtons[IMU][1].setText('Y')
			self.axisButtons[IMU][2].setText('Z')
			self.x_label.setText('a_x:\n\n(Start to View)')
			self.y_label.setText('a_y:\n\n(Start to View)')
			self.z_label.setText('a_z:\n\n(Start to View)')
			self.sensor.config_imu(2,0)
			self.sensor.config_imu(4,0)
			self.sensor.config_imu(5,0)
			self.sensor.config_imu(8,0)
		elif name == 'Linear Acceleration':
			self.imuValLabel = 'a'
			self.imu_y_label = 'Linear Acceleration (m/s^2)'
			self.update.emit(2, ['Linear Acceleration (m/s^2)'], XYZ)
			self.axisButtons[IMU][0].setText('X')
			self.axisButtons[IMU][1].setText('Y')
			self.axisButtons[IMU][2].setText('Z')
			self.x_label.setText('a_x:\n\n(Start to View)')
			self.y_label.setText('a_y:\n\n(Start to View)')
			self.z_label.setText('a_z:\n\n(Start to View)')
			self.sensor.config_imu(1,0)
			self.sensor.config_imu(4,0)
			self.sensor.config_imu(5,0)
			self.sensor.config_imu(8,0)
		elif name == 'Gyroscope':
			self.imuValLabel = 'g'
			self.imu_y_label = 'Gyroscope (Rad/s)'
			self.update.emit(2, ['Gyroscope (Rad/s)'], XYZ)
			self.axisButtons[IMU][0].setText('X')
			self.axisButtons[IMU][1].setText('Y')
			self.axisButtons[IMU][2].setText('Z')
			self.x_label.setText('g_x:\n\n(Start to View)')
			self.y_label.setText('g_y:\n\n(Start to View)')
			self.z_label.setText('g_z:\n\n(Start to View)')
			self.sensor.config_imu(1,0)
			self.sensor.config_imu(2,0)
			self.sensor.config_imu(5,0)
			self.sensor.config_imu(8,0)
		elif name == 'Rotation Vector': 
			self.imuValLabel = 'r'
			self.imu_y_label = 'Unit Quaternion'
			self.update.emit(2, ['Unit Quaternion'], ABC)
			self.axisButtons[IMU][0].setText('q_i')
			self.axisButtons[IMU][1].setText('q_j')
			self.axisButtons[IMU][2].setText('q_k')
			self.x_label.setText('q_i:\n\n(Start to View)')
			self.y_label.setText('q_j:\n\n(Start to View)')
			self.z_label.setText('q_k:\n\n(Start to View)')
			self.sensor.config_imu(1,0)
			self.sensor.config_imu(2,0)
			self.sensor.config_imu(4,0)
			self.sensor.config_imu(8,0)
		elif name == 'Game Rotation Vector':
			self.imuValLabel = 'r'
			self.imu_y_label = 'Unit Quaternion'
			self.update.emit(2, ['Unit Quaternion'], ABC)
			self.axisButtons[IMU][0].setText('q_i')
			self.axisButtons[IMU][1].setText('q_j')
			self.axisButtons[IMU][2].setText('q_k')
			self.x_label.setText('q_i:\n\n(Start to View)')
			self.y_label.setText('q_j:\n\n(Start to View)')
			self.z_label.setText('q_k:\n\n(Start to View)')
			self.sensor.config_imu(1,0)
			self.sensor.config_imu(2,0)
			self.sensor.config_imu(4,0)
			self.sensor.config_imu(5,0)

##################################################################################################################################
##################################################################################################################################
##################################################################################################################################

class PlotThread(QtCore.QThread):
	new_data = QtCore.pyqtSignal(list,list,float,list)
	finished1 = QtCore.pyqtSignal(list, list, list)

	def __init__(self, parent, sensor, log, rate, tabs, baseTab, funcTab, imuTab):
		QtCore.QThread.__init__(self)
		self.read = True
		self.tabs = tabs
		self.baseTab = baseTab
		self.funcTab = funcTab
		self.imuTab = imuTab
		self.sensor = sensor

		parent.update.connect(self.update_something)
		sensor.thread.data_source.connect(self.data_subscriber)
		self.plotCount = 0
		self.rate = rate
		self.fps = 20
		self.plotInterval = self.rate / self.fps
		self.log = log

		self.plotStyle = 0 #0 = scrolling, 1 = fixed
		self.keepSeconds = [2]*6 #How many seconds of data to keep in scrolling or fixed-frame plot
		self.keepSeconds[5] = 30

		self.toPlot = [[0,0,0],[0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0],[1]]
		self.colors = [(255,0,0),(0,0,255),(0,255,0),(251,0,255),(102,76,31),(200,140,16)]
		
		self.diff_data = []
		self.sum_data = []
		self.imu_data = []
		self.ft = []
		self.temp = []
		self.plot_data = [[],[],[],[],[],[]]

		self.vertices = np.array([[1, -1, -1],[-1, -1, -1],[-1, 1, -1],[-1, -1, 1],[1, 1, -1],[1, 1, 1],[-1, 1, 1],[1, -1, 1]]) #Centred at origin
		self.faces = np.array([[1,0,7],[1,3,7],[1,2,4],[1,0,4],[1,2,6],[1,3,6],[0,4,5],[0,7,5],[2,4,5],[2,6,5],[3,6,5],[3,7,5]])
		self.faceColors = np.array([[255,255,0,1],[255,255,0,1],[255,255,255,1],[255,255,255,1],[0,0,255,1],[0,0,255,1],[255,255,255,1],[255,255,255,1],[255,255,255,1],[255,255,255,1],[0,255,0,1],[0,255,0,1]])
		self.meshData = gl.MeshData(vertexes=self.vertices, faces=self.faces, faceColors=self.faceColors)
		self.plots = []
		self.pltwidgets = []
		
		leftAxes = ["Force (N)", "Torque (N.mm)", "Voltage (V)","Voltage (V)", parent.imu_y_label,"Temperature (C)"]
		self.titles = ["Force","Torque","Difference Signal","Sum Signal","IMU Reading","Temperature"]
		lengthList = [3,3,6,6,3,1]
		# Create plots
		for i in range(6):
			plotList=[]
			self.pltwidgets.append(pg.PlotWidget())
			for j in range(lengthList[i]):
				plotList.append(self.pltwidgets[i].plotItem.plot(width=2, pen=pg.mkPen(self.colors[j], width=2)))
			self.plots.append(plotList)
			self.pltwidgets[i].plotItem.showGrid(x=True,y=True)
			xMin = 0
			xMax = self.keepSeconds[0] * self.fps
			self.pltwidgets[i].plotItem.setLabels(left=leftAxes[i], bottom="Time (sec)", top=self.titles[i]) 

		self.pltwidgets[5].enableAutoRange(self.pltwidgets[5].plotItem.getViewBox().YAxis)
		self.setYRange = True
		self.oldYMin = 0
		self.oldYMax = 1

		# Add plots to tabs 
		self.baseTab.layout.addWidget(self.pltwidgets[FORCE],1,4,11,11)
		self.baseTab.layout.addWidget(self.pltwidgets[TORQUE],13,4,4,11)
		self.funcTab.layout.addWidget(self.pltwidgets[DIFF],1,2,11,9)
		self.funcTab.layout.addWidget(self.pltwidgets[SUM],13,2,9,9)
		self.imuTab.layout.addWidget(self.pltwidgets[IMU],1,4,11,11)
		self.imuTab.layout.addWidget(self.pltwidgets[TEMP],13,4,4,11)

		# Orientation plots
		self.orientation_disp = []
		self.cube = gl.GLMeshItem(meshdata=self.meshData, drawEdges=True, drawFaces=True)
		self.axis= gl.GLAxisItem()
		self.transform = QtGui.QMatrix4x4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0)

		for i in range(2):
			self.orientation_disp.append(gl.GLViewWidget())
			self.orientation_disp[i].addItem(self.cube)
			self.orientation_disp[i].addItem(self.axis)
			self.orientation_disp[i].setCameraPosition(distance=7)

		self.axis.setSize(x=2,y=2,z=2)
		self.baseTab.layout.addWidget(self.orientation_disp[0],13,0,4,4)
		# self.imuTab.layout.addWidget(self.orientation_disp[1],0,4,8,4)
		self.imuTab.layout.addWidget(self.orientation_disp[1],13,0,4,4)	

		self.limit = -1
		self.showOrientation=True

	def run(self):
		failCount = 0
		while self.read:
			if self.plotCount == self.limit:
				self.plotCount=0
				self.read == False
				self.sensor.stop_data_transmission()
				break
			time.sleep(0.5)
		
		self.finished1.emit(self.diff_data, self.sum_data, self.temp)
		self.diff_data = []
		self.sum_data = []
		self.imu_data = []
		self.ft = []
		self.temp = []
		self.plot_data = [[],[],[],[],[],[]]
		
	@QtCore.pyqtSlot(int, list, bool)
	def update_something(self, cmdNum, msg, state):
		if cmdNum == 0: #Update which curves to plot
			self.update_toPlot(msg[0],msg[1],state)
		if cmdNum == 1: #Update how many seconds to plot
			self.update_frame_length(msg[0],msg[1])
		if cmdNum == 2: #Update IMU y-axis label
			self.pltwidgets[IMU].plotItem.setLabels(left=msg[0], bottom='Time (sec)', top=self.titles[IMU])
		if cmdNum == 3: #Poll a single measurement
			self.poll(msg[0])
		if cmdNum == 4: #Set logging to True/False
			self.log = state
		if cmdNum == 5:
			self.set_disp_orientation(state)
		if cmdNum == 6:
			if state:
				print('Updating logging sample limit to ' + str(msg[0]))
				self.limit = msg[0]
			else:
				self.limit = None

	def poll(self, sample):

		if sample.quaternion != None:		
			self.update_rotation(sample.rotation)
			self.cube.setTransform(self.transform)

		self.diff_data.append(sample.differential)
		self.sum_data.append(sample.sum)
		self.imu_data.append(sample.imu)
		self.ft.append(sample.wrench)
		self.plot_data[0].append(sample.wrench[:3])
		self.plot_data[1].append(sample.wrench[3:])
		self.plot_data[2].append(sample.differential)
		self.plot_data[3].append(sample.sum)
		self.plot_data[4].append(sample.imu[:3])
		self.plot_data[5].append(sample.temperature)

		self.plotData()

	def set_disp_orientation(self, state):
		self.showOrientation = state

	def update_rotation(self, R):
		self.transform = QtGui.QMatrix4x4(R[0,0],R[0,1],R[0,2],0,R[1,0],R[1,1],R[1,2],0,R[2,0],R[2,1],R[2,2],0,0,0,0,1)

	def update_frame_length(self, idx, keepSeconds):
		self.keepSeconds[idx] = keepSeconds
		self.pltwidgets[idx].plotItem.getViewBox().setXRange(min=0,max=self.keepSeconds[idx])

	def update_toPlot(self, plotNum, index, state):
		self.toPlot[plotNum][index] = 1 if state else 0
		self.setYRange = True
		if state:
			self.pltwidgets[plotNum].addItem(self.plots[plotNum][index])
		else:
			self.pltwidgets[plotNum].removeItem(self.plots[plotNum][index])
		self.plotData()

	@QtCore.pyqtSlot(object)
	def data_subscriber(self, data):
		if self.read:
			if data != None and not data.failed():
				# data.string(detailed=True)
				
				self.plotCount += 1
				# Save data in arrays
				self.diff_data.append(data.differential)
				self.sum_data.append(data.sum)
				self.imu_data.append(data.imu)
				self.ft.append(data.wrench)
				self.temp.append(data.temperature)

				# if not self.log:
				self.new_data.emit(data.imu,data.wrench,data.temperature,data.saturated)

				if self.plotCount % self.plotInterval == 0:
					
					if not self.log:
						
						try:
							self.plot_data[0].append(data.wrench[:3])
							self.plot_data[1].append(data.wrench[3:])
							self.plot_data[2].append(data.differential)
							self.plot_data[3].append(data.sum)
							self.plot_data[4].append(data.imu)
							self.plot_data[5].append(data.temperature)
						except TypeError:
							print('Ignoring an error')
							pass
						#Check data length
						for i in range(6):
							dataLength = self.keepSeconds[i] * self.fps
							if len(self.plot_data[i]) > dataLength*5:
								self.plot_data[i] = self.plot_data[i][-dataLength:]

						if data.quaternion != None and self.showOrientation:
							self.update_rotation(data.rotation)
							self.cube.setTransform(self.transform)

						self.plotData()				
					else:
						pass
			else:
				print('None read')

	def plotData(self):
		plts = [[0,1],[2,3],[4,5]]
		ranges = [3,3,6,6,3,1]
		idx = self.tabs.currentIndex()

		for plt in plts[idx]:
			moving = False
			for i in range(ranges[plt]):
				if self.toPlot[plt][i] == 1:
					
					dataLength = self.keepSeconds[plt] * self.fps
					dataLength = dataLength if len(self.plot_data[plt]) > dataLength else len(self.plot_data[plt])
					x = np.linspace(0,self.keepSeconds[plt] if len(self.plot_data[plt]) >= self.keepSeconds[plt] * self.fps else float(len(self.plot_data[plt]))/self.fps,dataLength)
					
					if plt == TEMP:
						self.plots[plt][i].setData(x,self.plot_data[plt][-dataLength:])
					else:
						self.plots[plt][i].setData(x,[a[i] for a in self.plot_data[plt][-dataLength:]]) 
						yMin, yMax = self.getMinMax(plt,dataLength,ranges[plt])
						self.pltwidgets[plt].plotItem.getViewBox().setYRange(min=yMin,max=yMax)
					moving = True
			if moving == False:
				self.pltwidgets[plt].plotItem.getViewBox().setXRange(min=0,max=self.keepSeconds[plt])
				self.pltwidgets[plt].plotItem.getViewBox().setYRange(min=-1,max=1)

	def getMinMax(self, plotNum, dataLength, numAxes):
		mx = -1e10
		mn = 1e10
		for idx in range(numAxes):
			if self.toPlot[plotNum][idx] == 1:
				for i in range(dataLength):
					if self.plot_data[plotNum][i][idx] > mx:
						mx = self.plot_data[plotNum][i][idx]
					if self.plot_data[plotNum][i][idx] < mn:
						mn = self.plot_data[plotNum][i][idx]

		if mx == -1e10 and mn == 1e10:
			return -1,1
		else:
			rng = abs(mx - mn)*0.3
			if rng == 0:
				return mn-0.1, mx+0.1
			return mn-rng, mx+rng
           
class CalibrationThread(QtCore.QThread):

	def __init__(self, sensor, disp, indicator,parent):
		QtCore.QThread.__init__(self)
		self.read = True
		self.disp = disp
		self.sensor = sensor
		parent.stopThread.connect(self.stop_thread)
		self.count = 0
		self.quality = {0:'Unreliable', 1:'Low', 2:'Medium', 3:'High'}
		self.grState = -1
		self.magState = -1
		self.indicator = indicator
		self.on = False

	def run(self):
		while self.read:
			state, string = self.sensor.parse_calibration_state()
			print(str(state) + ': ' + string)
			if state != -1:
				try:
					if string == 'GRVEC_STS':
						self.grState = state
					elif string == 'MAG_STS':
						self.magState = state

					self.disp.setText('Game Rotation Accuracy: ' + self.quality[self.grState] + '\nMagnetometer Accuracy: ' + self.quality[self.magState])
					if self.count % 10 == 0:
						if self.on:
							self.indicator.setText('')
							self.on = False
						else:
							self.indicator.setText('Calibrating')
							self.on = True
				except KeyError:
					print('GR: ' + str(self.grState) + ', Mag: ' + str(self.magState))
			else:
				if self.count % 10 == 0:
					self.disp.setText('')
				else:
					self.disp.setText('No data')

			self.count += 1

	@QtCore.pyqtSlot(int)
	def stop_thread(self, n):
		print('Stopping')
		self.read = False

class CalibrationWindow(QMainWindow):
	stopThread = QtCore.pyqtSignal(int)

	def __init__(self, sensor):
		super(QWidget, self).__init__()

		self.title = "IMU Calibration Control Panel"
		self.left = 750
		self.top = 400
		self.width = 500
		self.height = 400
		self.setWindowTitle(self.title)
		self.setGeometry(self.left, self.top, self.width, self.height)
		self.setCentralWidget(QtGui.QWidget(self))

		self.layout = QVBoxLayout()
		self.centralWidget().setLayout(self.layout)

		self.sensor = sensor

		self.disp = QLabel('')
		self.start_button = QPushButton('Start Calibration')
		self.stop_button = QPushButton('Finish Calibration')
		self.cancel_button = QPushButton('Cancel Calibration')
		self.indicator = QLabel('')

		self.start_button.clicked.connect(self.start)
		self.stop_button.clicked.connect(self.stop)
		self.cancel_button.clicked.connect(self.cancel)

		self.layout.addWidget(self.indicator)
		self.layout.addWidget(self.disp)
		self.layout.addWidget(self.start_button)
		self.layout.addWidget(self.stop_button)
		self.layout.addWidget(self.cancel_button)

		self.calibrationThread = CalibrationThread(sensor, self.disp, self.indicator, self)

	def start(self):
		print('Starting calib')
		self.sensor.imu_start_calibration()
		self.calibrationThread.read = True
		self.calibrationThread.start()

	def stop(self):
		self.sensor.imu_finish_calibration()
		self.stopThread.emit(0)
		if self.calibrationThread.isRunning():
			time.sleep(1)
		if self.calibrationThread.isRunning():
			print('Caution, failed to stop calibration thread')

	def cancel(self):
		self.sensor.imu_cancel_calibration()
		self.stopThread.emit(0)
		if self.calibrationThread.isRunning():
			time.sleep(1)
		if self.calibrationThread.isRunning():
			print('Caution, failed to stop calibration thread')


class MainObject:
	def __init__(self, quick):
		signal.signal(signal.SIGINT, self.signal_handler)
		
		# try:
		app = QApplication([])
		self.GUI = App(quick)
		app.exec_()
		# except Exception as e:
		# 	print(e)

	def signal_handler(self, sig, frame):
		print('Killing Application...')
		self.GUI.control_widget.sensor.stop_data_transmission()
		sys.exit(0)

if __name__ == "__main__":
	quick = False
	if len(sys.argv) > 1:
		if sys.argv[1] == '-q':
			quick = True
	
	m = MainObject(quick)
	m.GUI.control_widget.sensor.stop_data_transmission()
	
