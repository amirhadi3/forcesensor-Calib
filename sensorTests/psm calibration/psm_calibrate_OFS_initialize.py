from sensor import Sensor
import time
import numpy as np

DES_CMMD = 2

calMatrix = np.eye(6)
sensor = Sensor(calMatrix)

#Turn on LEDs
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