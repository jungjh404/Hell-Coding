#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import *
from sensor_msgs.msg import LaserScan

class LaneScan:
    def __init__(self):
        self.prev_lyp_mid = 0
        self.prev_ryp_mid = 0
        self.image = np.empty(shape=[0])
        self.bridge = CvBridge()
        self.warp_img_w = 640
        self.warp_img_h = 480
        
        self.lane_pub_flag = True

        self.nwindows = 10
        self.margin = 30
        self.minpix = 5
        self.lane_bin_th = 180#180

        Width, Height = 640, 480
        self.warp_src = np.array([
            [Width*1/5 + 30, Height*1/2 + 40],
            [0,Height-82],
            [Width*4/5 - 15, Height*1/2 + 40],
            [Width,Height-82]
        ], dtype=np.float32)
        #marker
        
        self.warp_dist = np.array([
            [-40,0],
            [40, Height], #0
            [Width+40,0],
            [Width-40, Height]
        ], dtype=np.float32)

        rospy.wait_for_message("/usb_cam/image_rect", Image)
        
        self.scan_pub = rospy.Publisher('lane_scan', LaserScan, queue_size=50)
        rospy.Subscriber("/usb_cam/image_rect", Image, self.img_callback)
        rospy.loginfo("Lane to Scan Initialized!")
        self.time = time.time()


    def lane_pub(self, llx , rlx, lly, rly, lx_pix, rx_pix, ly_pix, ry_pix):
        num_readings = 2160 #1000 does not warp angle
        laser_frequency = 40

        error = 0.035 

        theta_l = np.arctan2(lly, llx)
        theta_l = np.where(theta_l > 0, theta_l, theta_l + np.pi)
        dist_l = np.sqrt(lly**2 + llx**2)
        theta_r = np.arctan2(rly, rlx)
        theta_r = np.where(theta_r > 0, theta_r, theta_r + np.pi)
        dist_r = np.sqrt(rlx**2 + rly**2)

        dict = {name:value for name, value in zip(theta_l, dist_l)}
        dict_r = {name:value for name, value in zip(theta_r, dist_r)}

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

        smp1 = s1[::self.cnt(s1)]
        smp2 = s2[::self.cnt(s2)]
        smp3 = s3[::self.cnt(s3)]
        smp4 = s4[::self.cnt(s4)]
        # cnt(np.array(s1))
        smp_fin = smp1 + smp2 + smp3 + smp4
        # print(len(smp1))
        # print(len(smp_fin))
        rmp1 = p1[::self.cnt(p1)]
        rmp2 = p2[::self.cnt(p2)]
        rmp3 = p3[::self.cnt(p3)]
        rmp4 = p4[::self.cnt(p4)]

        rmp_fin = rmp1 + rmp2 + rmp3 + rmp4
        
        current_time = rospy.Time.now()
        
        scan = LaserScan()

            # range is in [m]
        scan.header.stamp = current_time
        scan.header.frame_id = 'lane_scan'
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

        lp_mid = np.median(lx_pix)
        rp_mid = np.median(rx_pix)
        lyp_mid = np.median(ly_pix)
        ryp_mid = np.median(ry_pix)

        try:
            if np.isnan(lyp_mid):
                lyp_mid = self.prev_lyp_mid
        except NameError:
            pass

        try:
            if np.isnan(ryp_mid):
                ryp_mid = self.prev_ryp_mid
        except NameError:
            pass

        flag = 1
        flug = 1

        if  (rx_pix[0]-rx_pix[-1] > 25 and abs(ryp_mid -240) < 40): #or (abs(ryp_mid -240) < 40)):#and (math.isnan(ryp_mid) is False)): #(320-lp_mid > rp_mid-320)
            flug = 3
            for i in range(0, num_readings):
                if (k < len(rmp_fin) and abs(i * scan.angle_increment - rmp_fin[k][0]< error )):
                    try:
                        idx = int(rmp_fin[k][0] / math.pi * num_readings)
                        scan.ranges[idx] = rmp_fin[k][1]
                        scan.intensities[idx] = 1
                        k+=1
                    except IndexError:
                        pass
        
        elif (lx_pix[0]-lx_pix[-1] < -30 and abs(lyp_mid -240) < 40): #and (lyp_mid > 300): #(320-lp_mid < rp_mid-320)
            flag = 3
            for i in range(0, num_readings):
                if (j < len(smp_fin) and abs(i * scan.angle_increment -smp_fin[j][0]) < error ):
                    try:
                        idx = int(smp_fin[j][0] / math.pi * num_readings)
                        scan.ranges[idx] = smp_fin[j][1]
                        # print(smp_fin[j][1])
                        scan.intensities[idx] = 1
                        j+=1
                    except IndexError:
                        pass
        #elif flag == 1:
        #    print("no lane")
        else:
            pass
        
        if flag != 3 and (rp_mid - lp_mid > 450) and (200 < lyp_mid < 280):
            for i in range(0, num_readings):
                    #if (j == len(x)):
                    #    break

                    #if theta is within the increments
                if (j < len(smp_fin) and abs(i * scan.angle_increment -smp_fin[j][0]) < error ):
                    try:
                        idx = int(smp_fin[j][0] / math.pi * num_readings)
                        scan.ranges[idx] = smp_fin[j][1]
                        scan.intensities[idx] = 1
                        j+=1
                    except IndexError:
                        pass
                    #if use pass, there is time delay, resulting in error
        else:
            #print("left is false", lp_mid, rp_mid, lyp_mid, ryp_mid)
            flag = 1
        
        if flug != 3 and (rp_mid - lp_mid > 450) and (200 < ryp_mid < 280):
            flag = 0    
            for i in range(0, num_readings):
                if (k < len(rmp_fin) and abs(i * scan.angle_increment - rmp_fin[k][0]< error )):
                    try:
                        idx = int(rmp_fin[k][0] / math.pi * num_readings)
                        scan.ranges[idx] = rmp_fin[k][1]
                        # print(rmp_fin[k][1])
                        scan.intensities[idx] = 1
                        k+=1
                    except IndexError:
                        pass
        else:
            #print("right is false", lp_mid, rp_mid, lyp_mid, ryp_mid)
            pass
        
        if self.lane_pub_flag:
            self.scan_pub.publish(scan)

        self.prev_lyp_mid = lyp_mid
        self.prev_ryp_mid = ryp_mid


    def cnt(self, arr):
        r = 0
        if (len(arr)/50) == 0:
            r = 1
        else:
            r = len(arr)/50
        return r

    def img_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        warp_img, _, _ = self.warp_image(image, self.warp_src, self.warp_dist, (self.warp_img_w, self.warp_img_h))
        _, _, lxp, rxp, lyp, ryp = self.warp_process_image(warp_img)
        self.lane_pub(lxp, rxp, lyp, ryp, self.lx, self.rx, self.lly, self.rly)
        # cv2.imshow("out_img", self.out_img)
        cv2.waitKey(1)


    def warp_image(self, img, src, dst, size):
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

        return warp_img, M, Minv


    def warp_process_image(self, img):
        blur = cv2.GaussianBlur(img,(5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)   )
        lane = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
        self.out_img = np.dstack((lane, lane, lane))*255
        # cv2.imshow("lane", lane)

        histogram = np.sum(lane[lane.shape[0]//2:,:],   axis=0)      
        midpoint = np.int(histogram.shape[0]/2)
        leftx_current = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) +  midpoint

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
        window_height = np.int(lane.shape[0]/self.nwindows)
        nz = lane.nonzero()

        left_lane_inds = []
        right_lane_inds = []

        self.lx, self.ly, self.rx, self.ry = [], [], [], []
        
        for window in range(self.nwindows):
            
            win_yl = lane.shape[0] - (window+1)*window_height
            win_yh = lane.shape[0] - window*window_height

            win_xll = leftx_current - self.margin
            win_xlh = leftx_current + self.margin
            win_xrl = rightx_current - self.margin
            win_xrh = rightx_current + self.margin

            cv2.rectangle(self.out_img,(win_xll,win_yl),(win_xlh,    win_yh),    (0,255,0), 2) 
            cv2.rectangle(self.out_img,(win_xrl,win_yl),(win_xrh,    win_yh),    (0,255,0), 2) 

            good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh) & (nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh) & (nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.minpix:
                leftx_current = np.int(np.mean(nz[1][good_left_inds]))
            if len(good_right_inds) > self.minpix:        
                rightx_current = np.int(np.mean(nz[1][good_right_inds]))

            
            self.lx.append(leftx_current)
            self.ly.append((win_yl + win_yh)/2)

            self.rx.append(rightx_current)
            self.ry.append((win_yl + win_yh)/2)


        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        lfit = np.polyfit(np.array(self.ly),np.array(self.lx),2)
        rfit = np.polyfit(np.array(self.ry),np.array(self.rx),2)

        self.out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        self.out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]]= [0, 0, 255]    
        
        self.llx = nz[1][left_lane_inds]
        self.lly = nz[0][left_lane_inds]
        self.rlx = nz[1][right_lane_inds]
        self.rly = nz[0][right_lane_inds]
        llx_pix = self.x_pix(np.array(self.llx))
        rlx_pix = self.x_pix(np.array(self.rlx))
        lly_pix = self.y_pix(np.array(self.lly))
        rly_pix = self.y_pix(np.array(self.rly))

        return lfit, rfit, llx_pix, rlx_pix, lly_pix, rly_pix

    def x_pix(self, a):
        return (a - 320)*0.00129
    
    def y_pix(self, a):
        return (480 - a + 413)*0.00075
    


if __name__ == '__main__':
    rospy.init_node('lane_to_scan', annoymous=True)
    try:
        a = LaneScan()
    except KeyboardInterrupt:
        exit(0)
