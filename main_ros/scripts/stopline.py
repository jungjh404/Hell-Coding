#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import signal
import sys
import os
import random
from hell_coding.msg import IsStop



def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#==============================================
# ???��?????? ????? ????, ??????? ?????
#==============================================
image = np.empty(shape=[0]) # ???? ??????? ???? ????
bridge = CvBridge() 
motor = None # ???? ?????? ???? ????

#=============================================
# ???��?????? ????? ??? ?????
#=============================================
CAM_FPS = 30    # ???? FPS - ??? 30???? ?????? ????
Width, Height = 640, 480    # ???? ????? ????x???? ???

#=============================================
# ?????? - ???? ?????? ?????? ??????
# ???? ????? ?????? ??????? ??????? ????? ???
# ??????? ????? ?????? ???? image ?????? ??? ????.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def warp_image(img, src, dst, size):
    # ī�޶󿡼� �޾ƿ� �̹����� ������ ��ȯ
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv


################################testing#############################################


def detect(img):
    
    #return True if stopline is detected else False
    
    bev = img
    blur = cv2.GaussianBlur(bev, (5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    lane = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
    #_, lane = cv2.threshold(L, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow("lane", lane)
    
    num_white_pix = int(np.sum(lane == 255))
    num_black_pix = int(np.sum(lane == 0))
    total_num_of_pix = num_white_pix + num_black_pix
    min_threshold = 0.20

    print('Number of white pixels:', num_white_pix)
    print('Number of black pixels:', num_black_pix)
    #print('Percentage of black pixels:' , float(num_black_pix/total_num_of_pix*100))
    #print('Percentage of white pixels:' , float(num_white_pix/total_num_of_pix*100))
    print(total_num_of_pix * min_threshold)
    detected = False

    if num_white_pix > total_num_of_pix * min_threshold:
        detected = True
        print("detected")
        flag = 1
        rospy.Publisher("is_stop", IsStop, queue_size = 1)
        sys.exit()
    
    return detected        

####################################################################################



def start():

    global motor, image

    rospy.init_node('stopline')
    image_sub = rospy.Subscriber("/usb_cam/image_rect/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # ???�� ???? ?????? ?????? ?????? ????.
    while not image.size == (Width * Height * 3):
        continue

    #=========================================
    # ???? ???? 
    # ???? ?????? ??????? ??? ???? ????? ?????? ???? 
    # "???????? +?????????? +???????? +???????????" 
    # ????? ????????? ??????.
    #=========================================
    while not rospy.is_shutdown():

        # ?????????? ???? ???? ??????????? img?? ???? ????
        warp_img_w = 640
        warp_img_h = 480

        global nwindows
        global margin
        global minpix
        global lane_bin_th
        global dist
        global cal_mtx
        global cal_roi
        global mtx

        nwindows = 10
        margin = 30
        minpix = 5
        lane_bin_th = 180#180

        warp_det_src = np.array([
            [Width*1/5 + 30, Height*1/2+40],
            [0,Height-10],
            [Width*4/5 - 15, Height*1/2+40],
            [Width,Height-30]
        ], dtype = np.float32)


        warp_det_dist = np.array([
            [-40,0],
            [40, Height], #0
            [Width+40,0],
            [Width-40, Height]
        ], dtype = np.float32)



        warp_det_img, _, _ = warp_image(image, warp_det_src, warp_det_dist, (warp_img_w, warp_img_h))
        
        detect(warp_det_img)
        cv2.imshow("warp_det_img", warp_det_img)
        cv2.waitKey(1)
        #cv2.imshow("warp_img", warp_img)

if __name__ == '__main__':
    start()