import serial
import time

while(1):
	ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.5)
	ts = int(time.time())
	ser.write('ts' + 'Hello\r\n')
	time.sleep(0.1)
	ser.close()
