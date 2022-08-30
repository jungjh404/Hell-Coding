#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import signal
import sys
import os


def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

bridge = CvBridge() 
CAM_FPS = 30 
Width, Height = 640, 480 

class Stopline:
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.detected = False
        self.start()


    def img_callback(self, data):
        self.image = bridge.imgmsg_to_cv2(data, "bgr8")


    def warp_image(self, img, src, dst, size):
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)
        return warp_img, M, Minv


    def detect(self, img):
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
        
        return detected


    def start(self):
        rospy.Subscriber("/usb_cam/image_rect/",Image, self.img_callback)
        while not self.image.size == (Width * Height * 3):
            continue
        
        rospy.loginfo("Stopline Initialized!")

        while not rospy.is_shutdown():
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

            warp_det_img, _, _ = self.warp_image(self.image, warp_det_src, warp_det_dist, (Width, Height))
            
            self.stop = self.detect(warp_det_img)
            cv2.imshow("warp_det_img", warp_det_img)
            cv2.waitKey(1)
            #cv2.imshow("warp_img", warp_img)


if __name__ == '__main__':
    a = Stopline()