import pylibftdi as ftdi
import time
import struct
import crc
import codecs
from sensor import Sensor
import sensor_output as out
import numpy as np
import matplotlib.pyplot as plt
import datetime
import ati
from datetime import datetime as date
import matplotlib.pyplot as plt
import dvrk
import sys
import rospy
from std_msgs.msg import Float64, String
import os

#SOME FUNCTIONS
def toBytesList(string):
        if len(string) % 2 != 0:
            string = '0' + string
        return [int(string[i:i+2],16) for i in range(0,len(string),2)]
def toHex(num):
        return "%x" % num
def read_in(s,n):
    data = s.read(n)
    while len(data) != n:
        data += s.read(n-len(data))
    data = codecs.encode(data,'hex') #Read n byte packet
    data = toBytesList(data)
    return data
def toStr(byte_list):
        string = b''
        for i in range(len(byte_list)):
                    string += struct.pack("B", byte_list[i])
        string += struct.pack("B",crc.crc8(byte_list))
        return string
def makeTimestamp():
        d = date.now()
        return str(d)[:10] + '_' + str(d)[11:13] + '-' +  str(d)[14:16] + '-' + str(d)[17:19]
def to_int(byte):
    num = 0
    sz = len(byte)
    for i in range(sz):
        num += (byte[sz - 1 - i] << i*8)
    return num

class recorder(object):
    def __init__ (self,psm_name):
        rospy.init_node(str(psm_name+'_recorder'))
	self.insSubscriber = rospy.Subscriber(psm_name+'/insertion',Float64,self.__subCallback)
        self.statPublisher = rospy.Publisher(psm_name+'/recState',String, queue_size = 1)
        self.recState = String()
        self.__initOFS()
        self.__initATI()
        self.__initdVRK(psm_name)
	self.posData = []
	self.cartPosData = []
        self.cartOrnData = []
	self.velData = []
	self.effData = []
	self.diff = []
	self.sums = []
	self.temp = []
	self.atiData = []
        self.inc = 0
        self.insertion = 0
	self.insIncrement = 0.001;
	self.numdatPoints = 0

    def __initOFS(self):
	self.calMatrix = np.eye(6)
	sensor = Sensor(self.calMatrix)
	# test all modules are working
	time.sleep(1)
	sample = sensor.poll()
	for cmmd in sample.sum:
	    if cmmd < 0.5:
	        cont = False
	        print('Sum Signal too low (' + str(cmmd) + 'V)')
	sensor.disconnect()

        self.rep_ids = {
            0x1: 'Accelerometer',
            0x2: 'Gyroscope',
            0x4: 'Linear Acceleration',
            0x5: 'Rotation Vector',
            0x8: 'Game Rotation Vector'
            }
        self.crc8_table = crc.calculate_CRC8_table()
        self.crc32_table = crc.calculate_CRC32_table()

    
    def __initATI(self):
	self.atiSensor = ati.Ati('/home/david/Documents/forcesensor', 'FT24093',1)

    def __initdVRK(self,psm_name):
        self.arm = dvrk.psm(psm_name)
    
    def __subCallback(self,msg):
	self.insertion = msg.data

    def read(self):
	self.posData = []
	self.cartPosData = []
        self.cartOrnData = []
	self.effData = []
	self.diff = []
	self.sums = []
	self.temp = []
	self.atiData = []
	sPos = self.insertion
	outer_timeout = 45*1000*10
	while (abs(self.insertion - sPos) < self.insIncrement/2) and (outer_timeout != 0):
	    timeout = 100
            d = self.s.read(1)
            while d != b'\xAA' and timeout != 0:
                d = self.s.read(1)
                timeout -= 1
        
            if timeout == 0:
                outer_timeout -= 1
                continue

            d = codecs.encode(self.s.read(2),'hex')#Read 2 bytes (counter and crc)
            if len(d)==4 and int(d[2:],16) == crc.crc8(int('aa' + d[:2],16), table=self.crc8_table): #Check CRC 8
                data = read_in(self.s,53)
                if len(data)==53 and to_int(data[49:]) == crc.crc32(data[:49], table=self.crc32_table): #Check CRC 32
                    try:
                        rid = self.rep_ids.get(data[36])
                    except KeyError:
                        print('Invalid report ID key')

                    dat = out.sensor_output(None, data, rid, self.calMatrix)
    		    pos = np.append(self.arm.get_current_joint_position(), self.arm.get_current_jaw_position())
		    cartesianPos = self.arm.get_current_position()
		    cartPos = [float(i) for i in cartesianPos.p]
		    cartOr = cartesianPos.M.GetQuaternion()
                    vel = np.append(self.arm.get_current_joint_velocity(), self.arm.get_current_jaw_velocity())
    		    eff = np.append(self.arm.get_current_joint_effort(), self.arm.get_current_jaw_effort())
    		    self.posData.append(pos)
		    self.cartPosData.append(cartPos)
		    self.cartOrnData.append(cartOr)
		    self.velData.append(vel)
    		    self.effData.append(eff)
    		    self.atiData.append(self.atiSensor.read())
		    self.temp.append(dat.temperature)
                    self.diff.append(dat.differential)
                    self.sums.append(dat.sum)
                else:
                    print('CRC-32 failed')
            else:
                print('CRC-8 failed')

	if outer_timeout == 0:
	    print('outer timeout')
        self.numdatPoints+=len(self.diff) 

    def startFTDI(self,BAUD,HZ,PORT):
	#Open connection to port
	self.s = ftdi.Device(device_id = 'FT0NG8XX', interface_select=PORT)
	self.s.baudrate=BAUD
	self.s.ftdi_fn.ftdi_set_latency_timer(1)
	self.s.ftdi_fn.ftdi_set_line_property(8,1,0)

	#Generate byte message with sample rate 
	hz = toHex(HZ)
	
	while not len(hz) == 4:
	    hz = '0' + hz
	    b = b'' + toStr([0x10,int(hz[:2],16),int(hz[2:],16)])

    	#Start timing, write, and read
	self.startt = time.time()
	self.s.write(b) 
    	print('Started transmission')
    	sys.stdout.flush()
	self.recState.data = 'Go'
        self.statPublisher.publish(self.recState)
	self.recState.data = 'Stop'

    def stopFTDI(self):
	self.recState.data = 'Stop'
        self.statPublisher.publish(self.recState)
        self.s.write(b'\x11\x00\x00\xC9') #stop transmission
        #print('read and saved+ ' + str(self.numdatPoints) + ' packets in ' + str(time.time()-self.startt) + 'sec')
        count = 1
        while count < 100 and self.s.read(1) != '':
            self.s.write(b'\x11\x00\x00\xC9') #stop transmission
            count += 1
            self.s.ftdi_fn.ftdi_usb_purge_buffers()
        if count == 100:
            print('Failed to stop. Still sending data.')
	self.purge()
	self.s.close()

    def purge_rx(self,verbose=True):
	if self.s.ftdi_fn.ftdi_usb_purge_rx_buffer() == 0:
	    if verbose:
		print("Receive buffer cleared")
		return 0
	    else:
		print("Error clearing receive buffer")
		return -1
    def purge_tx(self, verbose = True):
	if self.s.ftdi_fn.ftdi_usb_purge_tx_buffer() == 0:
	    if verbose:
		print("Transmit buffer cleared")
		return 0
	    else:
		print("Error clearing transmit buffer")
		return -1

    def purge(self, verbose=True):
	return self.purge_tx(verbose) + self.purge_rx(verbose)

    def save(self):
	root, dirs, files = os.walk('/home/david/Desktop/').next()
	if dirs:
	    self.inc = max([int(i) for i in dirs])+1

        path = '/home/david/Desktop/'+str(self.inc)
	try:
    	    os.mkdir(path)
        except OSError:
    	    print ("Creation of the directory %s failed" % path)

    	np.savetxt(path+'/atiData.txt',np.array(self.atiData))
    	np.savetxt(path+'/diffData.txt',np.array(self.diff))
    	np.savetxt(path+'/sumData.txt',np.array(self.sums))
    	np.savetxt(path+'/tempData.txt',np.array(self.temp))
    	np.savetxt(path+'/cartPosData.txt',np.array(self.cartPosData))
    	np.savetxt(path+'/cartOrnData.txt',np.array(self.cartOrnData))
	np.savetxt(path+'/posData.txt',np.array(self.posData))
    	np.savetxt(path+'/velData.txt',np.array(self.velData))
    	np.savetxt(path+'/effData.txt',np.array(self.effData))

    def initCMMD(self):
	DES_CMMD = 2

	sensor = Sensor(self.calMatrix)

	for i in range(6):
	    current_command = 0
	    sensor.config_dac(i, current_command)
	    sample = sensor.poll()
    
	    while sample.sum[i] < DES_CMMD:
	        current_command += 1
	        sensor.config_dac(i, current_command)
	        sample = sensor.poll()

	    while sample.sum[i] > DES_CMMD:
	        current_command -= 0.1
	        sensor.config_dac(i, current_command)
	        sample = sensor.poll()

	    while sample.sum[i] < DES_CMMD:
	        current_command += 0.01
	        sensor.config_dac(i, current_command)
	        sample = sensor.poll()

	print "**************************************"
	print "Common-mode values of channels A to F:"
	print sample.sum
	print "**************************************"

	sensor.disconnect()
    
if __name__ == "__main__":
    rec = recorder('PSM2')
    while(rec.insertion == 0):
	pass
    while rec.insertion > 0.065:
	rec.initCMMD()
	rec.startFTDI(BAUD = 3000000, HZ = 1000, PORT = 2)
	rec.read()
	rec.stopFTDI()
	rec.save()
	time.sleep(0.5)





