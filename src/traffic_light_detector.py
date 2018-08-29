#!/usr/bin/env python
# coding: utf-8
# created by myan@makeblock.com 
# Date: 2018/08/19 
# Time: 15:20
#
import os
import sys
import cv2
import getopt
import numpy as np
import threading
import time
import serial
import serial.tools.list_ports
import re
from threading import Timer
import random
import codecs, binascii

# 全局变量定义
TRAFFIC_LIGHT_STATE_NULL = 0
TRAFFIC_LIGHT_STATE_RED = 1
TRAFFIC_LIGHT_STATE_GREEN = 2
TRAFFIC_LIGHT_STATE_YELLOW = 3

RED_COLOR      = 0
GREEN_COLOR    = 1
YELLOW_COLOR   = 2
LOWER          = 0
UPPER          = 1

timeCount = 0
mouse_clicked = False
traffic_light_state = TRAFFIC_LIGHT_STATE_NULL

red_light_max_radius_r = 0
red_light_max_radius_g = 0
red_light_max_radius_y = 0

red_lower_h_value = 0
red_lower_s_value = 100
red_lower_v_value = 210
red_upper_h_value = 10
red_upper_s_value = 255
red_upper_v_value = 255

green_lower_h_value = 40
green_lower_s_value = 50
green_lower_v_value = 210
green_upper_h_value = 90
green_upper_s_value = 255
green_upper_v_value = 255


yellow_lower_h_value = 15
yellow_lower_s_value = 150
yellow_lower_v_value = 220
yellow_upper_h_value = 35
yellow_upper_s_value = 255
yellow_upper_v_value = 255

red_minDist = 80
red_operator_threshold = 50
red_accumulator_threshold = 25
red_min_radius = 15
red_max_radius = 100

green_minDist = 60
green_operator_threshold = 50
green_accumulator_threshold = 25
green_min_radius = 15
green_max_radius = 100

yellow_minDist = 30
yellow_operator_threshold = 50
yellow_accumulator_threshold = 25
yellow_min_radius = 15
yellow_max_radius = 100

red_hs_threshold = 50
green_hs_threshold = 80
yellow_hs_threshold = 50

def max_value(x,y,z):
    index = 0
    max = x
    if y > max:
        max = y
        index = 2
    if z > max:
        max = z
        index = 3
    if max == 0:
        index = 0
    return max, index

def main(argv=None):
    try:
        timeCountTask = threading.Thread(target = timeCount_task)
        timeCountTask.start()
        imageProcessTask = threading.Thread(target = imageProcess_task)
        imageProcessTask.start()
        uartDataProcessTask = threading.Thread(target = uartDataProcess_task)
        uartDataProcessTask.start()
    except:
        print("error code main!")

def uartDataProcess_task():
    global traffic_light_state
    global red_lower_h_value
    global red_lower_s_value
    global red_lower_v_value
    global red_upper_h_value
    global red_upper_s_value
    global red_upper_v_value

    global green_lower_h_value
    global green_lower_s_value
    global green_lower_v_value
    global green_upper_h_value
    global green_upper_s_value
    global green_upper_v_value

    global yellow_lower_h_value
    global yellow_lower_s_value
    global yellow_lower_v_value
    global yellow_upper_h_value
    global yellow_upper_s_value
    global yellow_upper_v_value

    global red_minDist
    global red_operator_threshold
    global red_accumulator_threshold
    global red_min_radius
    global red_max_radius

    global green_minDist
    global green_operator_threshold
    global green_accumulator_threshold
    global green_min_radius
    global green_max_radius
 
    global yellow_minDist
    global yellow_operator_threshold
    global yellow_accumulator_threshold
    global yellow_min_radius
    global yellow_max_radius

    global red_hs_threshold
    global green_hs_threshold 
    global yellow_hs_threshold
    ser = None
    pre_port_serial = None 
    while True:
        t0 = time.time()
        port_list = list(serial.tools.list_ports.comports())
        if len(port_list) <= 0:
            print "None port find!!"
        else:
            for i in range(len(port_list)):
                port_list_0 = list(port_list[i])
                port_serial = port_list_0[0]
                if re.match('/dev/ttyUSB',port_serial) != None:
                    try:
                        if pre_port_serial != port_serial:
                            ser = serial.Serial(port_serial, 115200, timeout = 0.05)
                        if(ser is not None):
                            pre_port_serial = port_serial
                            data_frame = ser.readline()
                            if data_frame != "":
                                print data_frame
                                if (data_frame[0] == 'g') or (data_frame[0] == 'G'):
                                    cmd_data =  data_frame.split()
                                    if int(cmd_data[0][1:]) == 11:
                                        #print "set_color_ranger"
                                        if(cmd_data[1][0] == 'C') or (cmd_data[1][0] == 'c'):
                                            if int(cmd_data[1][1:]) == RED_COLOR:
                                                if(cmd_data[2][0] == 'T') or (cmd_data[2][0] == 't'):
                                                    if int(cmd_data[2][1:]) == LOWER:
                                                        for i in range(3,len(cmd_data)):
                                                            if(cmd_data[i][0] == 'H') or (cmd_data[i][0] == 'h'):
                                                                red_lower_h_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'S') or (cmd_data[i][0] == 's'):
                                                                red_lower_s_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'V') or (cmd_data[i][0] == 'v'):
                                                                red_lower_v_value = int(cmd_data[i][1:])
                                                    elif int(cmd_data[2][1:]) == UPPER:
                                                        for i in range(3,len(cmd_data)):
                                                            if(cmd_data[i][0] == 'H') or (cmd_data[i][0] == 'h'):
                                                                red_upper_h_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'S') or (cmd_data[i][0] == 's'):
                                                                red_upper_s_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'V') or (cmd_data[i][0] == 'v'):
                                                                red_upper_v_value = int(cmd_data[i][1:])
                                            elif int(cmd_data[1][1:]) == GREEN_COLOR:
                                                if(cmd_data[2][0] == 'T') or (cmd_data[2][0] == 't'):
                                                    if int(cmd_data[2][1:]) == LOWER:
                                                        for i in range(3,len(cmd_data)):
                                                            if(cmd_data[i][0] == 'H') or (cmd_data[i][0] == 'h'):
                                                                green_lower_h_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'S') or (cmd_data[i][0] == 's'):
                                                                green_lower_s_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'V') or (cmd_data[i][0] == 'v'):
                                                                green_lower_v_value = int(cmd_data[i][1:])
                                                    elif int(cmd_data[2][1:]) == UPPER:
                                                        for i in range(3,len(cmd_data)):
                                                            if(cmd_data[i][0] == 'H') or (cmd_data[i][0] == 'h'):
                                                                green_upper_h_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'S') or (cmd_data[i][0] == 's'):
                                                                green_upper_s_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'V') or (cmd_data[i][0] == 'v'):
                                                                green_upper_v_value = int(cmd_data[i][1:])
                                            elif int(cmd_data[1][1:]) == YELLOW_COLOR:
                                                if(cmd_data[2][0] == 'T') or (cmd_data[2][0] == 't'):
                                                    if int(cmd_data[2][1:]) == LOWER:
                                                        for i in range(3,len(cmd_data)):
                                                            if(cmd_data[i][0] == 'H') or (cmd_data[i][0] == 'h'):
                                                                yellow_lower_h_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'S') or (cmd_data[i][0] == 's'):
                                                                yellow_lower_s_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'V') or (cmd_data[i][0] == 'v'):
                                                                yellow_lower_v_value = int(cmd_data[i][1:])
                                                    elif int(cmd_data[2][1:]) == UPPER:
                                                        for i in range(3,len(cmd_data)):
                                                            if(cmd_data[i][0] == 'H') or (cmd_data[i][0] == 'h'):
                                                                yellow_upper_h_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'S') or (cmd_data[i][0] == 's'):
                                                                yellow_upper_s_value = int(cmd_data[i][1:])
                                                            if(cmd_data[i][0] == 'V') or (cmd_data[i][0] == 'v'):
                                                                yellow_upper_v_value = int(cmd_data[i][1:])
                                    elif int(cmd_data[0][1:]) == 12:
                                        #print "set_hough_circles_para"
                                        if(cmd_data[1][0] == 'C') or (cmd_data[1][0] == 'c'):
                                            if int(cmd_data[1][1:]) == RED_COLOR:
                                                for i in range(2,len(cmd_data)):
                                                    if(cmd_data[i][0] == 'D') or (cmd_data[i][0] == 'd'):
                                                        red_minDist = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'O') or (cmd_data[i][0] == 'o'):
                                                        red_operator_threshold = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'A') or (cmd_data[i][0] == 'a'):
                                                        red_accumulator_threshold = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'I') or (cmd_data[i][0] == 'i'):
                                                        red_min_radius = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'X') or (cmd_data[i][0] == 'x'):
                                                        red_max_radius = int(cmd_data[i][1:])
                                            elif int(cmd_data[1][1:]) == GREEN_COLOR:
                                                for i in range(2,len(cmd_data)):
                                                    if(cmd_data[i][0] == 'D') or (cmd_data[i][0] == 'd'):
                                                        green_minDist = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'O') or (cmd_data[i][0] == 'o'):
                                                        green_operator_threshold = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'A') or (cmd_data[i][0] == 'a'):
                                                        green_accumulator_threshold = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'I') or (cmd_data[i][0] == 'i'):
                                                        green_min_radius = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'X') or (cmd_data[i][0] == 'x'):
                                                        green_max_radius = int(cmd_data[i][1:])
                                            elif int(cmd_data[1][1:]) == YELLOW_COLOR:
                                                for i in range(2,len(cmd_data)):
                                                    if(cmd_data[i][0] == 'D') or (cmd_data[i][0] == 'd'):
                                                        yellow_minDist = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'O') or (cmd_data[i][0] == 'o'):
                                                        yellow_operator_threshold = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'A') or (cmd_data[i][0] == 'a'):
                                                        yellow_accumulator_threshold = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'I') or (cmd_data[i][0] == 'i'):
                                                        yellow_min_radius = int(cmd_data[i][1:])
                                                    if(cmd_data[i][0] == 'X') or (cmd_data[i][0] == 'x'):
                                                        yellow_max_radius = int(cmd_data[i][1:])
                                    elif int(cmd_data[0][1:]) == 13:
                                        #print "set_hs_threshold"
                                        if(cmd_data[1][0] == 'C') or (cmd_data[1][0] == 'c'):
                                            if int(cmd_data[1][1:]) == RED_COLOR:
                                                if(cmd_data[2][0] == 'T') or (cmd_data[2][0] == 't'):
                                                    red_hs_threshold = int(cmd_data[2][1:])
                                            if int(cmd_data[1][1:]) == GREEN_COLOR:
                                                if(cmd_data[2][0] == 'T') or (cmd_data[2][0] == 't'):
                                                    green_hs_threshold  = int(cmd_data[2][1:])
                                            if int(cmd_data[1][1:]) == YELLOW_COLOR:
                                                if(cmd_data[2][0] == 'T') or (cmd_data[2][0] == 't'):
                                                    yellow_hs_threshold = int(cmd_data[2][1:])
                            #print("red",red_hs_threshold)
                            data_frame = ""
                            t1 = time.time()
                            #print(t1-t0)
                            send_data = "G20 L" + str(traffic_light_state) + "\r\n"
                            print send_data
                            ser.write(send_data)
                    except Exception, error:
                        print("Exception:" + str(Exception))
                        print("error:" + str(error))

def timeCount_task():
    global timeCount
    while( True ):
        timeCount = timeCount + 1
        time.sleep(0.1)
        # print(timeCount)

def imageProcess_task():
    try:
        global mouse_clicked
        global traffic_light_state

        global red_lower_h_value
        global red_lower_s_value
        global red_lower_v_value
        global red_upper_h_value
        global red_upper_s_value
        global red_upper_v_value
    
        global green_lower_h_value
        global green_lower_s_value
        global green_lower_v_value
        global green_upper_h_value
        global green_upper_s_value
        global green_upper_v_value
    
        global yellow_lower_h_value
        global yellow_lower_s_value
        global yellow_lower_v_value
        global yellow_upper_h_value
        global yellow_upper_s_value
        global yellow_upper_v_value
    
        global red_minDist
        global red_operator_threshold
        global red_accumulator_threshold
        global red_min_radius
        global red_max_radius
    
        global green_minDist
        global green_operator_threshold
        global green_accumulator_threshold
        global green_min_radius
        global green_max_radius
 
        global yellow_minDist
        global yellow_operator_threshold
        global yellow_accumulator_threshold
        global yellow_min_radius
        global yellow_max_radius
    
        global red_hs_threshold
        global green_hs_threshold 
        global yellow_hs_threshold

        r = 15
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.namedWindow('traffic_light')
        #cv2.namedWindow('raw_image')
        #cv2.namedWindow('hsv_image')
        #cv2.namedWindow('maskr')
        #cv2.namedWindow('maskg')
        #cv2.namedWindow('masky')

        cv2.setMouseCallback('traffic_light',onMouse)
        print 'Showing camera feed. click window or press any key to stop.'

        cameraCapture = cv2.VideoCapture(0)
        while False == cameraCapture.isOpened():
            cameraCapture = cv2.VideoCapture(0)

        success, frame = cameraCapture.read()
        frame_count = 0
        while success and not mouse_clicked:
            #t0 = time.time()
            #totalFrame = cameraCapture.get(cv2.CAP_PROP_POS_FRAMES)
            #print("total frame",totalFrame)
            success, frame = cameraCapture.read()
            result_image = frame
            if frame_count % 5 == 0:
                hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # color range
                lower_red1 = np.array([red_lower_h_value,red_lower_s_value,red_lower_v_value ])
                upper_red1 = np.array([red_upper_h_value ,red_upper_s_value ,red_upper_v_value ])
                lower_red2 = np.array([160,100,210])
                upper_red2 = np.array([180,255,255])
                lower_green = np.array([green_lower_h_value ,green_lower_s_value ,green_lower_v_value ])
                upper_green = np.array([green_upper_h_value ,green_upper_s_value ,green_upper_v_value ])
                lower_yellow = np.array([yellow_lower_h_value ,yellow_lower_s_value ,yellow_lower_v_value ])
                upper_yellow = np.array([yellow_upper_h_value ,yellow_upper_s_value ,yellow_upper_v_value ])
                mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
                maskg = cv2.inRange(hsv_image, lower_green, upper_green)
                masky = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
                maskr = cv2.add(mask1, mask2)

                size = frame.shape
                # print size

                r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, red_minDist,
                                   param1=red_operator_threshold, param2=red_accumulator_threshold, minRadius=red_min_radius, maxRadius=red_max_radius)

                g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, green_minDist,
                                     param1=green_operator_threshold, param2=green_accumulator_threshold, minRadius=green_min_radius, maxRadius=green_max_radius)

                y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, yellow_minDist,
                                     param1=yellow_operator_threshold, param2=yellow_accumulator_threshold, minRadius=yellow_min_radius, maxRadius=yellow_max_radius)

                red_light_max_radius_r = 0
                red_light_max_radius_g = 0
                red_light_max_radius_y = 0

                # traffic light detect
                if r_circles is not None:
                    r_circles = np.uint16(np.around(r_circles))

                    for i in r_circles[0, :]:
                        if i[0] > size[1] or i[1] > size[0] or i[1]+i[2] > size[0]:
                            continue
                        #print("r_circles detect!")
                        kernel = np.ones((i[2]/5,i[2]/5),np.uint8)
                        maskr = cv2.morphologyEx(maskr, cv2.MORPH_CLOSE, kernel)
                        h, s = 0.0, 0.0
                        r = int(i[2])
                        for m in range(-r, r):
                            for n in range(-r, r):
                                if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                                    continue
                                h += maskr[i[1]+m, i[0]+n]
                                s += 1
                        #print("r[h/s]=", h/s)
                        if h / s > red_hs_threshold:
                            cv2.circle(result_image, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                            cv2.circle(maskr, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                            cv2.putText(result_image,'RED',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)
                            if i[2] > red_light_max_radius_r:
                                red_light_max_radius_r = i[2]

                if g_circles is not None:
                    g_circles = np.uint16(np.around(g_circles))

                    for i in g_circles[0, :]:
                        if i[0] > size[1] or i[1] > size[0] or i[1] + i[2]> size[0]:
                            continue
                        kernel = np.ones((i[2]/5,i[2]/5),np.uint8)
                        maskg = cv2.morphologyEx(maskg, cv2.MORPH_CLOSE, kernel)
                        h, s = 0.0, 0.0
                        r = int(i[2])
                        for m in range(-r, r):
                            for n in range(-r, r):
                                if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                                    print("retur error!",h, s, r)
                                    continue
                                h += maskg[i[1]+m, i[0]+n]
                                s += 1
                        #print("g[h/s]=", h/s)
                        if h / s > green_hs_threshold :
                            cv2.circle(result_image, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                            cv2.circle(maskg, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                            cv2.putText(result_image,'GREEN',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)
                            if i[2] > red_light_max_radius_g:
                                red_light_max_radius_g = i[2]

                if y_circles is not None:
                    y_circles = np.uint16(np.around(y_circles))

                    for i in y_circles[0, :]:
                        if i[0] > size[1] or i[1] > size[0] or i[1] + i[2]> size[0]:
                            continue
                        kernel = np.ones((i[2]/5,i[2]/5),np.uint8)
                        masky = cv2.morphologyEx(masky, cv2.MORPH_CLOSE, kernel)
                        r = int(i[2])
                        h, s = 0.0, 0.0
                        for m in range(-r, r):
                            for n in range(-r, r):
                                if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                                    continue
                                h += masky[i[1]+m, i[0]+n]
                                s += 1
                        #print("y[h/s]=", h/s)
                        if h / s > yellow_hs_threshold:
                            cv2.circle(result_image, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                            cv2.circle(masky, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                            cv2.putText(result_image,'YELLOW',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)
                            if i[2] > red_light_max_radius_y:
                                red_light_max_radius_y = i[2]
            
                radius, traffic_light_state = max_value(red_light_max_radius_r,red_light_max_radius_g,red_light_max_radius_y)
                if traffic_light_state != TRAFFIC_LIGHT_STATE_NULL:
                    cv2.imshow('traffic_light',result_image)

            frame_count = frame_count + 1
            if frame_count == 4*100000:
                frame_count = 0
            if traffic_light_state == TRAFFIC_LIGHT_STATE_NULL:
                    cv2.imshow('traffic_light',result_image)
            #cv2.imshow('maskr', maskr)
            #cv2.imshow('maskg', maskg)
            #cv2.imshow('masky', masky)
            cv2.waitKey(10)
            #count = count + 1
            #print("test 1", count)
            #t1 = time.time()
            #print("time", t1 - t0)
        cv2.destroyWindow('traffic_light')
        #cv2.destroyWindow('raw_image')
        #cv2.destroyWindow('hsv_image')
        #cv2.destroyWindow('maskr')
        #cv2.destroyWindow('maskg')
        #cv2.destroyWindow('masky')
        cameraCapture.release()
    except Exception, error:
        print("error code imageProcess_task!")
        print("Exception:" + str(Exception))
        print("error:" + str(error))
        cv2.destroyWindow('traffic_light')
        #cv2.destroyWindow('raw_image')
        #cv2.destroyWindow('hsv_image')
        #cv2.destroyWindow('maskr')
        #cv2.destroyWindow('maskg')
        #cv2.destroyWindow('masky')
        cameraCapture.release()
        return -1

def onMouse(event,x,y,flags,param):
    global mouse_clicked
    if event == cv2.EVENT_RBUTTONUP:
        mouse_clicked = True

if __name__ == "__main__":
    sys.exit(main())
