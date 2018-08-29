#!/usr/bin/env python
import serial
from time import sleep
import serial.tools.list_ports
import re
import threading
from threading import Timer
import random
import os
import codecs, binascii


while True:
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) <= 0:
	print "None port find!!"
    else:
        for i in range(len(port_list)):
	    port_list_0 = list(port_list[i])
            port_serial = port_list_0[0]
	    print port_serial
            if re.match('/dev/ttyUSB',port_serial) != None:
                try:
	            ser = serial.Serial(port_serial, 115200, timeout = 0.5)
                    if(ser is not None):
                        print (' ... OK')
                        ser.write("\xF3\xF6\x03\x00\x0A\x00\x03\x0D\xF4")
                        sleep(0.2)
                        ser.close()
                except:
                    print("Write Error!!!")
    sleep(0.5)