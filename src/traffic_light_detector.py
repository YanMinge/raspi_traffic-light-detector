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

# 全局变量定义
TRAFFIC_LIGHT_STATE_NULL = 0
TRAFFIC_LIGHT_STATE_RED = 1
TRAFFIC_LIGHT_STATE_GREEN = 2
TRAFFIC_LIGHT_STATE_YELLOW = 3

timeCount = 0
mouse_clicked = False
traffic_light_state = TRAFFIC_LIGHT_STATE_NULL
red_light_max_radius_r = 0
red_light_max_radius_g = 0
red_light_max_radius_y = 0

def max_value(x,y,z):
    index = 1
    max = x
    if y > max:
        max = y
        index = 2
    if z > max:
        max = z
        index = 3
    return max, index

def main(argv=None):
    try:
        #timeCountTask = threading.Thread(target = timeCount_task)
        #timeCountTask.start()
        #imageProcessTask = threading.Thread(target = imageProcess_task)
        #imageProcessTask.start()
        uartDataProcessTask = threading.Thread(target = uartDataProcess_task)
        uartDataProcessTask.start()
    except:
        print("error code main!")

def uartDataProcess_task():
    global traffic_light_state
    while( True ):
        pass

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
            t0 = time.time()
            #totalFrame = cameraCapture.get(cv2.CAP_PROP_POS_FRAMES)
            #print("total frame",totalFrame)
            success, frame = cameraCapture.read()
            result_image = frame

            if frame_count % 5 == 0:
                hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # color range
                lower_red1 = np.array([0,100,210])
                upper_red1 = np.array([10,255,255])
                lower_red2 = np.array([160,100,210])
                upper_red2 = np.array([180,255,255])
                lower_green = np.array([40,50,210])
                upper_green = np.array([90,255,255])
                lower_yellow = np.array([15,150,220])
                upper_yellow = np.array([35,255,255])
                mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
                maskg = cv2.inRange(hsv_image, lower_green, upper_green)
                masky = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
                maskr = cv2.add(mask1, mask2)

                size = frame.shape
                # print size

                r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80,
                                   param1=50, param2=25, minRadius=15, maxRadius=100)

                g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, 60,
                                     param1=50, param2=25, minRadius=15, maxRadius=100)

                y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, 30,
                                     param1=50, param2=25, minRadius=15, maxRadius=100)

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
                        if h / s > 50:
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
                        if h / s > 100:
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
                        if h / s > 50:
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
            cv2.waitKey(5)
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
