See OneNote and comments in code for detailed documentation until it is finalized and written up more nicely

sensor.py: The class that controls the sensor through the USB-FTDI-RS485 connection.
sensor_output.py: Class to parse and store the outputs from the sensor in an understandable and useable manner
virtual_ftd2xx.py: A virtual 'sensor' which can control a separate COM port on computer and provide similar functionality and responses as the sensor
test_latency.py: Measure the roundtrip latency of sending and receiving a data packet through the FTDI chip using different python libraries to control it
test.py: Test functionality of sensor class