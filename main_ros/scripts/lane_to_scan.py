#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from xycar_msgs.msg import xycar_motor
from math import *
from sensor_msgs.msg import LaserScan



# from hell_coding.msg import _IsStop
class Publishint():
    def initfunc(self, llx , rlx, lly, rly):
        #rospy.init_node('laser_scan_publisher')

        scan_pub = rospy.Publisher('lane_scan', LaserScan, queue_size=50)
    
        num_readings = 2160 #1000 does not warp angle
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

        #####################################################################################
        dict = {name:value for name, value in zip(theta_l, dist_l)}
        dict_r = {name:value for name, value in zip(theta_r, dist_r)}
        #print(dict)
        #theta_l = np.array(theta_l)

        l1=0
        l2=0
        l3=0
        r1=0
        r2=0
        r3=0

        if len(theta_l) != 0:
            l1 = float(np.percentile(theta_l , 25, interpolation='linear'))
            l2 = float(np.percentile(theta_l , 50, interpolation='linear'))
            l3 = float(np.percentile(theta_l , 75, interpolation='linear'))
        if len(theta_r) != 0:
            r1 = float(np.percentile(theta_r , 25, interpolation='linear'))
            r2 = float(np.percentile(theta_r , 50, interpolation='linear'))
            r3 = float(np.percentile(theta_r , 75, interpolation='linear'))

        s1 = sorted({key:value for key, value in dict.items() if key < l1}.items()) # if key < l1
        s2 = sorted({key:value for key, value in dict.items() if key >= l1 and key < l2}.items())
        s3 = sorted({key:value for key, value in dict.items() if key >= l2 and key < l3}.items())
        s4 = sorted({key:value for key, value in dict.items() if key >= l3}.items())
        
        p1 = sorted({key:value for key, value in dict_r.items() if key < r1}.items())
        p2 = sorted({key:value for key, value in dict_r.items() if key >= r1 and key < r2}.items())
        p3 = sorted({key:value for key, value in dict_r.items() if key >= r2 and key < r3}.items())
        p4 = sorted({key:value for key, value in dict_r.items() if key >= r3}.items())

        #print(p1)

        smp1 = s1[::cnt(s1)]
        smp2 = s2[::cnt(s2)]
        smp3 = s3[::cnt(s3)]
        smp4 = s4[::cnt(s4)]
        
        smp_fin = smp1 + smp2 + smp3 + smp4
        print(len(smp1))
        print(len(smp_fin))
        rmp1 = p1[::cnt(p1)]
        rmp2 = p2[::cnt(p2)]
        rmp3 = p3[::cnt(p3)]
        rmp4 = p4[::cnt(p4)]

        rmp_fin = rmp1 + rmp2+ rmp3 + rmp4

        
        ###############################################################################
        
        current_time = rospy.Time.now()
        
        scan = LaserScan()

            # range is in [m]
        scan.header.stamp = current_time
        scan.header.frame_id = 'base_scan'
        scan.angle_min = -math.pi/2
        scan.angle_max = math.pi/2
        scan.angle_increment = math.pi / num_readings
        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
        scan.range_min = 0.0
        scan.range_max = 100.0
    
        scan.ranges = [0]*num_readings
        scan.intensities = [0]*num_readings


        

        j = 0
        k = 0
        
        for i in range(0, num_readings):
                #if (j == len(x)):
                #    break

                #if theta is within the increments
            if (j < len(smp_fin) and abs(i * scan.angle_increment -smp_fin[j][0]) < error ):
                try:
                    idx = int(smp_fin[j][0] / math.pi * num_readings)
                    scan.ranges[idx] = smp_fin[j][1]
                    print(smp_fin[j][1])
                    scan.intensities[idx] = 1
                    j+=1
                except IndexError:
                    pass
                #if use pass, there is time delay, resulting in error
        
        for i in range(0, num_readings):
            if (k < len(rmp_fin) and abs(i * scan.angle_increment - rmp_fin[k][0]< error )):
                try:
                    idx = int(rmp_fin[k][0] / math.pi * num_readings)
                    scan.ranges[idx] = rmp_fin[k][1]
                    print(rmp_fin[k][1])
                    scan.intensities[idx] = 1
                    k+=1
                except IndexError:
                    pass    
        
        scan_pub.publish(scan)

def cnt(arr):
    r = 0
    if (len(arr)/50) == 0:
        r = 1
    else:
        r = len(arr)/50
    return r



#=============================================
# ???��??? Ctrl-C ???????? ???��?? ?????? ???? ??
# ?? ????��??? ????? ???? ???
# #=============================================
# def signal_handler(sig, frame):
#     import time
#     time.sleep(3)
#     os.system('killall -9 python rosout')
#     sys.exit(0)

# signal.signal(signal.SIGINT, signal_handler)

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


def warp_image(img, src, dst, size):
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

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh) & (nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh) & (nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nz[1][good_right_inds]))

        
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


####################################################################################
################################testing#############################################

pix_threshold = 0.07

def detect(img):
    
    #return True if stopline is detected else False
    
    bev = img
    blur = cv2.GaussianBlur(bev, (5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    lane = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
    number_of_white_pix = np.sum(lane == 255)
    number_of_black_pix = np.sum(lane == 0)
    total_num_of_pix = number_of_white_pix + number_of_black_pix
    
    #print('Number of white pixels:', number_of_white_pix)
    #print('Number of black pixels:', number_of_black_pix)
    #print(total_num_of_pix * pix_threshold)
    detected = False

    if number_of_white_pix > pix_threshold*total_num_of_pix:
        detected = True
        print("detected")
        rospy.Publisher("is_stop", _IsStop, queue_size = 1)
    
    return detected          

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
    image_sub = rospy.Subscriber("/usb_cam/image_rect", Image, img_callback)
    
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
        detect(warp_det_img)

        global left_fit
        global right_fit

        left_fit, right_fit,lxp,rxp,lyp,ryp = warp_process_image(warp_img)
        
        pub.initfunc(lxp,rxp,lyp,ryp)
        
        cv2.imshow("out_img",out_img)
        cv2.imshow("warp_img", warp_img)
        cv2.waitKey(1)

        # r = rospy.Rate(10)
        # r.sleep()


#=============================================
# ???? ???
# ???? ???? ????? ????? ???? start() ????? ?????.
# start() ????? ???????? ???? ?????. 
#=============================================
if __name__ == '__main__':
    try:
        start()
    except KeyboardInterrupt:
        exit(0)
