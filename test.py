from sensor import sensor
import time

def pause_micro(us):
	for i in range(us*10):
		continue

d = sensor()
print('Reset device')
d.reset_device()
print('Reset DAC')
d.reset_dac()
print('Reset IMU')
d.reset_imu()
print('Reset transducers 1 and 4:')
d.reset_transducer(1)
d.reset_transducer(4)
print('Test invalid transducer number:')
d.reset_transducer(12)
print('Reset all transducers:')
d.reset_transducer(7)
print('Read in and parse 1 packet:')
d.measure().string()
print('Disable transducer 3')
d.deactivate_transducer(3)
print('Configure IMU')
d.set_imu_accelerometer(delay=10)
print('Configure DAC')
d.config_dac(3, 1)
print('Configure Transducer')
d.set_pga_gain(4, 10)

data = []
print('Start continuous data transmission')

d.start_data_transmission(1500)
time.sleep(1)
start = time.time()
for i in range(1500): #Read in the data live and do something with it
	data.append(d.read())
	print(str(data[i].differential))
	pause_micro(500)
d.stop_data_transmission()
stop = time.time()
del d

print(str(len(data)) + ' Data packets collected in ' + str((stop-start)*1000) + 'ms')

#import pdb; pdb.set_trace() #debugging'

