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

TIME = 20 #seconds

BAUD = 3000000
HZ = 1500
N_SAMPLES = TIME * HZ
PORT = 2
POLL = False
SAVE = True
DES_CMMD = 2
p = dvrk.psm("PSM2")

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

atiSensor = ati.Ati('/home/david/Documents/forcesensor', 'FT24093',1)
timestamp = makeTimestamp()
calMatrix = np.eye(6)
count=0
outer_timeout = N_SAMPLES * 10
rep_ids = {
            0x1: 'Accelerometer',
            0x2: 'Gyroscope',
            0x4: 'Linear Acceleration',
            0x5: 'Rotation Vector',
            0x8: 'Game Rotation Vector'
            }
crc8_table = crc.calculate_CRC8_table()
crc32_table = crc.calculate_CRC32_table()
diff = []
sums = []
atiData = []
temp = []
pos = []

sensor = Sensor(calMatrix)
fsc = [0]*6
ofc = [0]*6
#Turn on LEDs, read FSC, OFC registers

# test all modules are working
for i in range(6):
    sensor.config_dac(i,7)

cont = True
time.sleep(1)
sample = sensor.poll()
for cmmd in sample.sum:
    if cmmd < 0.5:
        cont = False
        print('Sum Signal too low (' + str(cmmd) + 'V)')

if cont:
    # set the common-mode signal of all the channels to the desired
    # common-mode value (DES_CMMD)
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

    #Open connection to port
    s = ftdi.Device(interface_select=PORT)
    s.baudrate=BAUD
    s.ftdi_fn.ftdi_set_latency_timer(1)
    s.ftdi_fn.ftdi_set_line_property(8,1,0)

    #Generate byte message with sample rate 
    hz = toHex(HZ)
    #print(hz)
    while not len(hz) == 4:
        hz = '0' + hz
    b = b'' + toStr([0x10,int(hz[:2],16),int(hz[2:],16)])

    #Start timing, write, and read
    startt = time.time()
    s.write(b) 
    print('Started transmission')


    times=[]
    while count < N_SAMPLES and outer_timeout != 0:
        timeout = 100
        start = time.time()
        d = s.read(1)
        while d != b'\xAA' and timeout != 0:
            d = s.read(1)
            timeout -= 1
        
        if timeout == 0:
            outer_timeout -= 1
            continue

        d = codecs.encode(s.read(2),'hex')#Read 2 bytes (counter and crc)
        if len(d)==4 and int(d[2:],16) == crc.crc8(int('aa' + d[:2],16), table=crc8_table): #Check CRC 8
            data = read_in(s,53)
            if len(data)==53 and to_int(data[49:]) == crc.crc32(data[:49], table=crc32_table): #Check CRC 32
                try:
                    rid = rep_ids.get(data[36])
                except KeyError:
                    print('Invalid report ID key')

                dat = out.sensor_output(None, data, rid, calMatrix)
                temp.append(dat.temperature)
                atiData.append(atiSensor.read())
                diff.append(dat.differential)
                sums.append(dat.sum)
                pos.append(p.get_current_joint_position()[2])

                count += 1
#               if count % (1500*5) == 0:
#                   print('5 seconds passed')
            else:
                print('CRC-32 failed')
        else:
            print('CRC-8 failed')
        outer_timeout -= 1

    s.write(b'\x11\x00\x00\xC9') #stop transmission
    print('read and saved+ ' + str(len(diff)) + ' packets in ' + str(time.time()-startt) + 'sec')
    count = 1
    while count < 100 and s.read(1) != '':
        s.write(b'\x11\x00\x00\xC9') #stop transmission
        count += 1
        s.ftdi_fn.ftdi_usb_purge_buffers()
    if count == 100:
        print('Failed to stop. Still sending data.')

    np.savetxt('/home/david/Desktop/atiData_' + timestamp+'.txt',np.array(atiData))
    np.savetxt('/home/david/Desktop/diffData_' + timestamp+'.txt',np.array(diff))
    np.savetxt('/home/david/Desktop/sumData_' + timestamp+'.txt',np.array(sums))
    np.savetxt('/home/david/Desktop/tempData_' + timestamp+'.txt',np.array(temp))
    np.savetxt('/home/david/Desktop/posData_' + timestamp+'.txt',np.array(pos))