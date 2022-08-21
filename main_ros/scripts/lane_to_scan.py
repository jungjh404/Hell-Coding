#! /usr/bin/env python
# -*- coding: utf-8 -*-

from turtle import left
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random
from sensor_msgs.msg import LaserScan
import pandas as pd  


from hell_coding.msg import _IsStop

class Publishint():
    def initfunc(self, llx , rlx, lly, rly):
        #rospy.init_node('laser_scan_publisher')

        scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    
        num_readings = 5000 #1000 does not warp angle
        laser_frequency = 40

        count = 0
        #r = rospy.Rate(0.1)
        #while not rospy.is_shutdown():
            # in [m]
            #right_x = [-0.1,0.2,0.3,0.4]
            #right_y = [0.1,0.3,0.5,0.7]
            
            # about 2 degrees
        error = 0.035 

        theta_l = []
        dist_l = []
        theta_r = []
        dist_r = []
        #if (len(llx) != 0 and int(len(llx)/50)!=0) :
            #print(len(llx),len(llx)/50)
        #for i in range( 0, len(llx), int((len(llx)/50)) ):
        for i in range( 0, len(llx) ):
            if (llx[i] > 0):
                theta_l.append(math.atan(lly[i]/llx[i]))
            else:
                theta_l.append(math.atan(lly[i]/llx[i]) + 3.14)
            dist_l.append(math.sqrt( math.pow(llx[i],2)+math.pow(lly[i],2) ))
        
        for i in range(0, len(rlx)):
            #theta_r.append(math.atan2(rly[i] , rlx[i]))
            if (rlx[i] > 0):
                theta_r.append(math.atan(rly[i]/rlx[i]))
            else:
                theta_r.append(math.atan(rly[i]/rlx[i]) + 3.14)
            dist_r.append(math.sqrt( math.pow(rlx[i],2)+math.pow(rly[i],2) ))
            # sort is need because lidar spins continously
            # if condition is not met, laserscan is not shown
        
        #theta_l.sort()
        
        dict = {'degree': theta_l, 'distance': dist_l}
        df = pd.DataFrame(dict)
        
        q_1 = df.degree.quantile(0.25)
        q_2 = df.degree.quantile(0.5)
        q_3 = df.degree.quantile(0.75)

        #subset by percentile
        sub1 = df[ df.loc[:,'degree'] < q_1].sort_values(by = 'degree', inplace = False)
        sub2 = df[ (df.loc[:,'degree'] >= q_1) & (df.loc[:,'degree'] < q_2) ].sort_values(by = 'degree', inplace = False)
        sub3 = df[ (df.loc[:,'degree'] >= q_2) & (df.loc[:,'degree'] < q_3) ].sort_values(by = 'degree', inplace = False)
        sub4 = df[ df.loc[:,'degree'] >= q_3].sort_values(by = 'degree', inplace = False)

        sub1.reset_index(drop = True)
        sub2.reset_index(drop = True)
        sub3.reset_index(drop = True)
        sub4.reset_index(drop = True)

        rate1 = int(sub1['degree'].count() / 50)
        rate2 = int(sub2['degree'].count() / 50)
        rate3 = int(sub3['degree'].count() / 50)
        rate4 = int(sub4['degree'].count() / 50)

        #prevent error when rate is 0
        if rate1 == 0:
            rate1 = 1
        if rate2 == 0:
            rate2 = 1
        if rate3 == 0:
            rate3 = 1
        if rate4 == 0:
            rate4 = 1

        #sample of about 50 points
        samp1 = sub1[::rate1].reset_index(drop = True)
        samp2 = sub2[::rate2].reset_index(drop = True)
        samp3 = sub3[::rate3].reset_index(drop = True)
        samp4 = sub4[::rate4].reset_index(drop = True)

        #can also concat right lanes to 'frames'
        #integrate into one dataframe
        frames = [samp1,samp2,samp3,samp4]
        samp_final_l = pd.concat(frames)
        #print(samp1.iloc[1][0])
        #print(samp_final.head())
        #print(samp_final.describe())
        #################################################################################

        dict_r = {'degree': theta_r, 'distance': dist_r}
        df_r = pd.DataFrame(dict_r)
        
        q_1r = df_r.degree.quantile(0.25)
        q_2r = df_r.degree.quantile(0.5)
        q_3r = df_r.degree.quantile(0.75)

        #subset by percentile
        sub1r = df_r[ df_r.loc[:,'degree'] < q_1r].sort_values(by = 'degree', inplace = False)
        sub2r = df_r[ (df_r.loc[:,'degree'] >= q_1r) & (df_r.loc[:,'degree'] < q_2r) ].sort_values(by = 'degree', inplace = False)
        sub3r = df_r[ (df_r.loc[:,'degree'] >= q_2r) & (df_r.loc[:,'degree'] < q_3r) ].sort_values(by = 'degree', inplace = False)
        sub4r = df_r[ df_r.loc[:,'degree'] >= q_3r].sort_values(by = 'degree', inplace = False)

        #df_r = df_r.sort_values(by = 'degree', inplace = False)
        #sub1r = df_r[ df_r.loc[:,'degree'] < q_1r]
        #sub2r = df_r[ (df_r.loc[:,'degree'] >= q_1r) & (df_r.loc[:,'degree'] < q_2r) ]
        #sub3r = df_r[ (df_r.loc[:,'degree'] >= q_2r) & (df_r.loc[:,'degree'] < q_3r) ]
        #sub4r = df_r[ df_r.loc[:,'degree'] >= q_3r]
        
        sub1r.reset_index(drop = True)
        sub2r.reset_index(drop = True)
        sub3r.reset_index(drop = True)
        sub4r.reset_index(drop = True)

        rate1_r = int(sub1r['degree'].count() / 50)
        rate2_r = int(sub2r['degree'].count() / 50)
        rate3_r = int(sub3r['degree'].count() / 50)
        rate4_r = int(sub4r['degree'].count() / 50)

        #prevent error when rate is 0
        if rate1_r == 0:
            rate1_r = 1
        if rate2_r == 0:
            rate2_r = 1
        if rate3_r == 0:
            rate3_r = 1
        if rate4_r == 0:
            rate4_r = 1

        #sample of about 50 points
        samp1_r = sub1r[::rate1_r].reset_index(drop = True)
        samp2_r = sub2r[::rate2_r].reset_index(drop = True)
        samp3_r = sub3r[::rate3_r].reset_index(drop = True)
        samp4_r = sub4r[::rate4_r].reset_index(drop = True)

        #can also concat right lanes to 'frames' 
        frames_r = [samp1_r,samp2_r,samp3_r,samp4_r]

        #print(frames_r)
        #print('---------------------')
        samp_final_r = pd.concat(frames_r)
        #samp_final_r = samp_final_r.sort_values(by='degree', inplace = False)
        #samp_final_r.reset_index(drop = True)

        #################################################################################

        current_time = rospy.Time.now()
    
        scan = LaserScan()

            # range is in [m]
        scan.header.stamp = current_time
        scan.header.frame_id = 'base_scan'
        scan.angle_min = 0
        scan.angle_max = 3.14
        scan.angle_increment = 3.14 / num_readings
        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
        scan.range_min = 0.0
        scan.range_max = 100.0
    
        scan.ranges = []
        scan.intensities = []

        # r, theta = np.meshgrid(dist_r, theta_r)
        templ=samp_final_l['degree'].values.tolist()
        tempr=samp_final_r['degree'].values.tolist()
        distl=samp_final_l['distance'].values.tolist()
        distr=samp_final_r['distance'].values.tolist()



        j = 0
        k = 0
        for i in range(0, num_readings):
                #if (j == len(x)):
                #    break

                #if theta is within the increments
            
            if (j < samp_final_l['degree'].count() and abs(i * scan.angle_increment -templ[j]) < error ):
                scan.ranges.append(distl[j])
                scan.intensities.append(1)
                j+=1
                #if use pass, there is time delay, resulting in error
                
                
            else:
                #is it fine to make huge to ignore?????
                scan.ranges.append(1)
                #scan.ranges.append(random.random())  # fake data
                scan.intensities.append(3)  # fake data
                #print(i * scan.angle_increment)

            
            if (k < samp_final_r['degree'].count() and abs(i * scan.angle_increment - tempr[k]< error )): #samp_final_r.iloc[k][0])
                scan.ranges.append(distr[k])#samp_final_r.iloc[k][1]
                scan.intensities.append(1)
                k+=1
                #if use pass, there is time delay, resulting in error
            else:
                #is it fine to make huge to ignore?????
                scan.ranges.append(1)
                #scan.ranges.append(random.random())  # fake data
                scan.intensities.append(3)  # fake data
                #print('else')
                #print(i * scan.angle_increment)
            
        scan_pub.publish(scan)





#=============================================
# ???��??? Ctrl-C ???????? ???��?? ?????? ???? ??
# ?? ????��??? ????? ???? ???
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#==============================================
# ???��?????? ????? ????, ??????? ?????map
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

def drive(angle, speed):

    global motor

    #motor_msg = xycar_motor()
    #motor_msg.angle = angle
    #motor_msg.speed = speed

    #motor.publish(motor_msg)

def warp_image(img, src, dst, size):
    # ī�޶󿡼� �޾ƿ� �̹����� ������ ��ȯ
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
    #_, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    lane = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
    cv2.rectangle(lane, (0,Height-80), (20,Height), (0,0,0), -1)
    cv2.rectangle(lane, (Width-20,Height-100), (Width+40,Height), (0,0,0), -1)

    cv2.imshow("lane", lane)

    histogram = np.sum(lane[lane.shape[0]//2:,:],   axis=0)      
    midpoint = np.int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) +  midpoint

    #lefty_current = 
    #righty_current = 
    #print(leftx_current - rightx_current)





    if (leftx_current >= midpoint):
        leftx_current = int(midpoint - 355 / 2)
                
    if rightx_current <= midpoint:
        rightx_current = int(midpoint + 355 / 2)
    if rightx_current - leftx_current < 300:
        if 320 - leftx_current < rightx_current - 320:
            leftx_current = int(midpoint - 355 / 2)
        else:
            rightx_current = int(midpoint + 355 / 2)

    #print(leftx_current, rightx_current)
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

   

    llx = []
    llx = nz[1][left_lane_inds]
    lly = []
    lly = nz[0][left_lane_inds]
    rlx = []
    rlx = nz[1][right_lane_inds]
    rly = []
    rly = nz[0][right_lane_inds]
    #print('------------------------------------')

    #print('left')
    #print(len(nz[1][left_lane_inds]))
    #print(len(nz[0][left_lane_inds]))

    #print('right')
    #print(len(nz[1][right_lane_inds]))
    #print(len(nz[0][right_lane_inds]))
    
    llx_pix = [x_pix(i) for i in llx]
    rlx_pix = [x_pix(i) for i in rlx]
    lly_pix = [y_pix(i) for i in lly]
    rly_pix = [y_pix(i) for i in rly]

    #print(llx_pix)
    #print('-----------------------')

    #print(len(llx_pix))
    #print(len(lly_pix))

    #print('right')
    #print(len(rlx_pix))
    #print(len(rly_pix))



    

    return lfit, rfit, llx_pix, rlx_pix, lly_pix, rly_pix

def x_pix(a):
    return (a - 320)*0.00145
def y_pix(a):
    return (480 - a + 107)*0.00145


def lane_tracking(left_line_angle, right_line_angle, curr_position):
    # Lane tracking algorithm here
    lane_angle = -(0.5*left_line_angle + 0.5*right_line_angle)

    k1 = 0.7  # P gain
    k2 = 400   # Distance parameter

    if -0.2 < curr_position < 0.2 :  # ?? ?? ???? ??????? ?????? ???
        k2 = 400
    
    elif curr_position > 0.5 or curr_position < -0.5 :  # ?????? ?????? ????? ??
        k2 = 250

    else:   # ?? ????? ??? ????? linear ???
        k2 = -500*abs(curr_position) + 500

    steer_angle = lane_angle * k1 + math.atan(-180 * (curr_position) / k2) * 180 / math.pi

    if steer_angle * lane_angle < 0:
        steer_angle = lane_angle * 2.5

    return steer_angle
####################################################################################
################################testing#############################################

red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)


stopline_threshold = 200 #125
area_threshold = 2000
length_threshold = 2000#300
'''  
def detect(img):
    
    #return True if stopline is detected else False
    
    bev = img
    blur = cv2.GaussianBlur(bev, (5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    #_, lane = cv2.threshold(L, stopline_threshold, 255, cv2.THRESH_BINARY)
    lane = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
    _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    bev_copy = bev.copy()
    #?��?��?�� ?���??�� 그려주기
    bev_copy = cv2.rectangle(bev_copy, (0,0), (Width, Height*3/4), (0,0,0), -1)
    cv2.waitKey(1)
    cv2.drawContours(image=bev_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    #bev_copy = cv2.rectangle(bev_copy, (0,0), (Width, Height*3/4), (0,0,0), -1)
    detected = False
    for cont in contours:
        
        length = cv2.arcLength(cont, True)
        area = cv2.contourArea(cont)
        
        #if not ((area > area_threshold) and (length > length_threshold)):
        #    continue
        #
        #if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
        #    continue
        
        #629, 406]], dtype=int32), array([[613, 371]], dtype=int32), array([[610, 406]], dtype=int32), array([[ 60, 425]], dtype=int32))
        
        x, y, w, h = cv2.boundingRect(cont)
        center = (x + int(w/2), y + int(h/2))
        _, width, _ = bev_copy.shape
        
        
        if (w<50):
            continue
        print(w,h)
        cv2.rectangle(bev_copy, (x, y), (x + w, y + h), blue, 2)
        if (w > 200) and (h > 200):
            cv2.rectangle(bev_copy, (x, y), (x + w, y + h), red, 2)
            p1, p2, p3, p4 = cv2.approxPolyDP(cont, length*0.02, True)
            print(p1,p2,p3,p4)
            
            detected = True
            rospy.Publisher("is_stop", _IsStop, queue_size = 1)
        
    if not detected:
        print("Lane is not detected")
    else:
        print("Lane is detected")
    #cv2.imshow('stopline', bev_copy)
    
    return detected        
'''
####################################################################################


#=============================================
# ??????? - ?????? ?????
#=============================================
def start():

    # ?????? ?????? ?????? start() ????? ???????? ??
    global motor, image

    #============================+=============
    # ROS ??? ??????? ???? ??.
    # ???? ?????? ??????? ???? ?????? ?????? ?????? ????
    #=========================================
    rospy.init_node('driving')
    #motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_rect/",Image,img_callback)
    
    print ("----- Xycar self driving -----")

    # ???�� ???? ?????? ?????? ?????? ????.
    while not image.size == (Width * Height * 3):
        continue


    pub = Publishint()

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

        warp_src  = np.array([
            [Width*1/5 - 30, Height*3/4],
            [0+5,Height-70],
            [Width*4/5, Height*3/4],
            [Width-5,Height-70]
        ], dtype=np.float32)

        #blue : left
        #image = cv2.circle(image, (Width*1/5, Height*3/4), 10, (255,0,0), 3)
        #image = cv2.circle(image, (0,Height-70), 10, (255,0,0), 3)
        #yellow : right
        #image = cv2.circle(image, (Width*4/5, Height*3/4), 10, (0,255,255), 3)
        #image = cv2.circle(image, (Width,Height-70), 10, (0,255,255), 3)
        
        warp_dist = np.array([
            [0+30,0],
            [0+5, Height-70], #0
            [Width-5,0],
            [Width, Height-70]
        ], dtype=np.float32)

        #left low
        #image = cv2.circle(image, (0,Height-60), 10, (0,0,255), 3)
        #right low
        #image = cv2.circle(image, (Width,Height-60), 10, (0,0,255), 3)
        
        #left high
        #image = cv2.circle(image, (Width/6,Height-120), 10, (0,0,255), 3)
        #right high
        #image = cv2.circle(image, (Width*5/6,Height-120), 10, (0,0,255), 3)
        cv2.imshow("original", image)
        #warp convertion array for def detect(stopline detection function)
        warp_det_src = np.array([
            [Width/6,Height-120],
            [0,Height-60],
            [Width,Height-60],
            [Width*5/6,Height-120]
        ], dtype = np.float32)


        warp_det_dist = np.array([
            [0,Height-180],
            [0,Height-60],
            [Width,Height-60],
            [Width,Height-180]
        ], dtype = np.float32)


        warp_img, _, _ = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
        warp_det_img, _, _ = warp_image(image, warp_det_src, warp_det_dist, (warp_img_w, warp_img_h))
        #detect(warp_det_img)
        #cv2.imshow("warp_det_img", warp_det_img)
        #cv2.imshow("warp_img", warp_img)

        global left_fit
        global right_fit

        left_fit, right_fit,lxp,rxp,lyp,ryp = warp_process_image(warp_img)

        
        pub.initfunc(lxp,rxp,lyp,ryp)



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
        
        # ??????? ???? ?????? ??????? ???��???
        cv2.putText(out_img, "angle: "+str(round(angle,1)), (240,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.putText(out_img, "speed: "+str(speed), (240,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        
        cv2.imshow("out_img",out_img)
        cv2.imshow("warp_img", warp_img)
        cv2.waitKey(1)


        drive(angle, speed)


#=============================================
# ???? ???
# ???? ???? ????? ????? ???? start() ????? ?????.
# start() ????? ???????? ???? ?????. 
#=============================================
if __name__ == '__main__':
    start()
    #sys.stdout.close()