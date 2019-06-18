from sensor import sensor
import time
import queue

d = sensor()
print('Reset device')
d.reset()
print('Reset DAC')
d.reset_dac()
print('Reset IMU')
d.reset_imu()
print('Reset transducers 1 and 4:')
d.reset_transducer(1)
d.reset_transducer(4)
print('Test invalid transducer number:')
d.reset_transducer(12)
print('Read in and parse 1 packet:')
d.request_data().print()


data = []
print('Start continuous data transmission')
start = time.time()
d.start_data_transmission()
for i in range(1500): #Read in the data live and do something with it
	data.append(d.read_packet())
	print(str(data[i].differential))
d.stop_data_transmission()
stop = time.time()
del d

print(str(len(data)) + ' Data packets collected in ' + str((stop-start)*1000) + 'ms')

#import pdb; pdb.set_trace() #debugging