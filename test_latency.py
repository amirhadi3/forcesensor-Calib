import pyftdi.serialext as serial
import time
from numpy import random

"""
Do some tests with latency using 2 different libraries
"""

#Create some random data
toSend = bytes(random.randint(0,10,size=51).tolist())

###### Using pyftdi.serialext:
#open 2 ports
# port = serial.serial_for_url('COM12', baudrate=5000000, timeout=0.1) #1ms timeout
# port2 = serial.serial_for_url('COM13', baudrate=5000000, timeout=0.1)
# print(port)

# #Measure round-trip latency of sending single byte and returning 51
# start = time.time()
# port.write(b'\xff')
# read1 = port2.read()
# port2.write(toSend)
# read2 = port.read(51)
# end = time.time()
# print("Elapsed Time: " + str((end-start)*1000) + "ms")

# #Ensure the correct data was read
# print(read1)
# print(read2)

# #Close the connections
# port.close()
# port2.close()


###### Using ftd2xx:
import ftd2xx
#Open 2 ports
port = ftd2xx.open(0)
port2 = ftd2xx.open(1)
port.setBaudRate(5000000)
port2.setBaudRate(5000000)

#Set latency timer and transfer size
port.setLatencyTimer(1)
port2.setLatencyTimer(1) #Makes a big difference. Default is 16ms, min is 1ms, 1ms is by far the fastest

port.setUSBParameters(64,64) #Makes no difference for some reason
port2.setUSBParameters(64,64)

#Measure round-trip latency of sending single byte and returning 51
start = time.time()
port.write(b'\xff')
read1 = port2.read(1)
port2.write(toSend)
read2 = port.read(51)
end = time.time()

print("Elapsed Time: " + str((end-start)*1000) + "ms")

#Ensure correct thing was measured
print(read1)
print(read2)

#Close ports
port2.close()
port.close()