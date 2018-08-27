#!/usr/bin/env python
# coding: utf-8
# created by myan@makeblock.com 
# Date: 2018/08/24
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
mouse_clicked = False

def main(argv=None):
	global mouse_clicked
	cv2.namedWindow('raw_image')

	#cv2.setMouseCallback('raw_image',onMouse)
	print 'Showing camera feed. click window or press exit key to stop.'
	cameraCapture = cv2.VideoCapture(0)
	success, frame = cameraCapture.read()

	while success and not mouse_clicked:
		success,frame = cameraCapture.read()
		raw_image = frame
		result_image = frame
		cv2.imshow('raw_image', raw_image)
		key = cv2.waitKey(10)
		if key != -1:
			print("key=")
			print(key)
		if(key == 27):
			break;
		if(key == 112):
			time_str = time.strftime("%H-%M-%S", time.gmtime())
			image_path = "./image/" + "image-"+ time_str + ".jpeg"
			print(image_path)
			cv2.imwrite(image_path, raw_image)

	cv2.destroyWindow('raw_image')
	cameraCapture.release()

def onMouse(event,x,y,flags,param):
    global mouse_clicked
    if event == cv2.EVENT_RBUTTONUP:
        mouse_clicked = True

if __name__ == "__main__":
	sys.exit(main())
