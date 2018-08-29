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
            #print port_serial
            if re.match('/dev/ttyUSB',port_serial) != None:
                try:
                    ser = serial.Serial(port_serial, 115200, timeout = 0.5)
                    if(ser is not None):
                        data_frame = ser.readline()
                        if data_frame != "":
                            print data_frame[0]
                            if (data_frame[0] == 'g') or (data_frame[0] == 'G'):
                                cmd_data =  data_frame.split()
                                if int(cmd_data[0][1:]) == 11:
                                    print "set_color_ranger"
                                    for i in range(1,len(cmd_data)):
                                        data_str = cmd_data[i][1:]
                                        print int(data_str)
                                elif int(cmd_data[0][1:]) == 12:
                                    print "set_hough_circles_para"
                                elif int(cmd_data[0][1:]) == 13:
                                    print "set_hs_threshold"
                        ser.write("G20 L1\r\n")
                        sleep(0.01)
                        ser.write("G20 L0\r\n")
                        sleep(0.01)
                        #ser.close()
                except Exception, error:
                    print("Exception:" + str(Exception))
                    print("error:" + str(error))
    sleep(0.5)