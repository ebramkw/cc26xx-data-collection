# Author: ebramkw
# This program is to feed the timestamp to the serial port "/dev/ttyACM$port_num$" every $period$
# Run: python send-time.py port_number period_in_seconds
# example to send to /dev/ttyACM0 every minute:
#     python send-time.py 0 60

import serial
from datetime import datetime
import time
import sys

# datetime_object = datetime.strptime('Dec 20 2018  11:59PM', '%b %d %Y %I:%M%p')

port_num = sys.argv[1]
period = sys.argv[2]

port = '/dev/ttyACM' + port_num

ser = serial.Serial(
   port=port,\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
print("connected to: " + ser.portstr)

while True:
   # if int(time.time()) <= int(time.mktime(datetime_object.timetuple())):
   # 	print "time sent: " + str(int(time.time()));
   # 	print "";
   # 	ser.write(str(int(time.time())) + "\r\n");
   # else:
   #    print "stop";
   #    print "";
   #    ser.write("1010101010\r\n");
	
   print "time sent: " + str(int(time.time()));
   ser.write(str(int(time.time())) + "\r\n");
   time.sleep(float(period));

ser.close()
