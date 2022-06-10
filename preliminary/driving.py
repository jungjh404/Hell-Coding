#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# �Բ� ���Ǵ� ���� ���̽� ��Ű������ import �����
#=============================================
from turtle import left
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random
#=============================================
# �͹̳ο��� Ctrl-C Ű�Է����� ���α׷� ������ ���� ��
# �� ó���ð��� ���̱� ���� �Լ�
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#==============================================
# ���α׷����� ����� ����, ������� �����
#==============================================
image = np.empty(shape=[0]) # ī�޶� �̹����� ���� ����
bridge = CvBridge() 
motor = None # ���� ������ ���� ����

#=============================================
# ���α׷����� ����� ��� �����
#=============================================
CAM_FPS = 30    # ī�޶� FPS - �ʴ� 30���� ������ ����
Width, Height = 640, 480    # ī�޶� �̹��� ����x���� ũ��

#=============================================
# �ݹ��Լ� - ī�޶� ������ ó���ϴ� �ݹ��Լ�
# ī�޶� �̹��� ������ �����ϸ� �ڵ����� ȣ��Ǵ� �Լ�
# ���ȿ��� �̹��� ������ ���� image ������ �Ű� ����.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

def warp_image(img, src, dst, size):
    # 카메라에서 받아온 이미지를 버드뷰로 전환한다
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv


def warp_process_image(img):
    global nwindows
    global margin
    global minpix
    global lane_bin_th

    blur = cv2.GaussianBlur(img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)   )
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.       THRESH_BINARY)

    histogram = np.sum(lane[lane.shape[0]//2:,:],   axis=0)      
    midpoint = np.int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) +  midpoint

    if (leftx_current == 0):
        leftx_current = int(midpoint - 355 / 2)
                      
    if rightx_current==midpoint:
        rightx_current = int(midpoint + 355 / 2)

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()

    left_lane_inds = []
    right_lane_inds = []

    global lx, ly, rx, ry
    lx, ly, rx, ry = [], [], [], []

    global out_img
    out_img = np.dstack((lane, lane, lane))*255

    for window in range(nwindows):
        
        win_yl = lane.shape[0] - (window+1)*window_height
        win_yh = lane.shape[0] - window*window_height

        win_xll = leftx_current - margin
        win_xlh = leftx_current + margin
        win_xrl = rightx_current - margin
        win_xrh = rightx_current + margin

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,    win_yh),    (0,255,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,    win_yh),    (0,255,0), 2) 

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&   (nz    [1] >= win_xll)&(nz[1] < win_xlh)).nonzero()    [0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)   &(nz   [1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()    [0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1]    [good_left_inds])   )
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nz[1]       [good_right_inds]))

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]]= [0, 0, 255]    

    return lfit, rfit

def lane_tracking(left_line_angle, right_line_angle, curr_position):
    # Lane tracking algorithm here
    lane_angle = -(0.5*left_line_angle + 0.5*right_line_angle)

    k1 = 0.7  # P gain
    k2 = 400   # Distance parameter

    if -0.2 < curr_position < 0.2 :  # �� �� õõ�� �����ص� ������ ��Ȳ
        k2 = 400
    
    elif curr_position > 0.5 or curr_position < -0.5 :  # �ż��ϰ� ����� ���;� ��
        k2 = 250

    else:   # �� �߰��� ��� ����� linear ��ȭ
        k2 = -500*abs(curr_position) + 500

    steer_angle = lane_angle * k1 + math.atan(-180 * (curr_position) / k2) * 180 / math.pi

    if steer_angle * lane_angle < 0:
        steer_angle = lane_angle * 2.5

    return steer_angle

#=============================================
# �����Լ� - ������ �����
#=============================================
def start():

    # ������ ������ ������ start() �ȿ��� ����ϰ��� ��
    global motor, image

    #=========================================
    # ROS ��带 �����ϰ� �ʱ�ȭ ��.
    # ī�޶� ������ �����ϰ� ���� ������ ������ ������ ����
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # ù��° ī�޶� ������ ������ ������ ��ٸ�.
    while not image.size == (Width * Height * 3):
        continue
 
    #=========================================
    # ���� ���� 
    # ī�޶� ������ �����ϴ� �ֱ⿡ ���� �ѹ��� ������ ���鼭 
    # "�̹���ó�� +������ġã�� +���Ⱒ���� +�������ȹ���" 
    # �۾��� �ݺ������� ������.
    #=========================================
    while not rospy.is_shutdown():

        # �̹���ó���� ���� ī�޶� �����̹����� img�� ���� ����
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
        lane_bin_th = 225

        warp_src  = np.array([
            [Width*1/5, Height*3/4 - 5],  
            [0,Height-60],
            [Width*4/5, Height*3/4 - 5],
            [Width,Height-60]
        ], dtype=np.float32)

        warp_dist = np.array([
            [128,0],
            [128,Height-60],
            [512,0],
            [512, Height-60]
        ], dtype=np.float32)
          
        mtx = np.array([
            [422.037858, 0.0, 245.895397], 
            [0.0, 435.589734, 163.625535], 
            [0.0, 0.0, 1.0]
        ])
        
        dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        
        cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist,(Width, Height), 1, (Width, Height))

        warp_img, _, _ = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))\

        global left_fit
        global right_fit

        left_fit, right_fit = warp_process_image(warp_img)

        left_fit = np.polyfit(np.array(ly),np.array(lx),1)
        right_fit = np.polyfit(np.array(ry),np.array(rx),1)
        
        line_left = np.poly1d(left_fit)
        line_right = np.poly1d(right_fit)
        
        left_line_angle = math.degrees(math.atan(line_left[1]))
        right_line_angle = math.degrees(math.atan(line_right[1]))
        
        shift_const = 355
   
        left_position = ((320 - float(lx[3])) / shift_const) * 2 - 1
        right_position = -((-320 + float(rx[4])) / shift_const) * 2 + 1
        
        position = 0
        
        if abs(math.degrees(math.atan(line_right[1]))) < 0.05:
            position = 0.9*left_position + 0.1*right_position
        
        elif abs(math.degrees(math.atan(line_left[1]))) < 0.05:
            position = 0.1*left_position + 0.9*right_position
        
        else:
            position = (left_position+right_position)/2
        
        angle = lane_tracking(left_line_angle, right_line_angle, position)               
        
        speed = 40
        
        if abs(angle) > 7:
            speed = 25
        
        # ������� ���� ����Ϳ� �̹����� ���÷���
        cv2.putText(out_img, "angle: "+str(round(angle,1)), (240,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.putText(out_img, "speed: "+str(speed), (240,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("CAM View", out_img)
        cv2.waitKey(1)


        drive(angle, speed)


#=============================================
# ���� �Լ�
# ���� ���� ȣ��Ǵ� �Լ��� ���⼭ start() �Լ��� ȣ����.
# start() �Լ��� �������� ���� �Լ���. 
#=============================================
if __name__ == '__main__':
    start()

