#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : xycar_motor.py
# 버 전 : v1.1
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
#############################################################################

import rospy
import time
import math
import sys
import os
import signal
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from goal_manager import GoalManager



def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class motor:
    def __init__(self, goal_manager, idx):
        deviceSetFunc = [self.set_vesc]
        self.control = False
        self.debug = False
        self.start_time = time.time()
        self.gm = None
        self.motor_type = 0
        self.stop_cnt = 0
        deviceSetFunc[self.motor_type]()
        self.set_parameter()
        self.ros_init(goal_manager=="true", idx)        
        rospy.spin()

    def set_parameter(self):
        motor_controller = [
            [0, 0, 0.0068, 0, 0, 0.08, 0, None, self.auto_drive_vesc]
        ]

        self.angle_weight = motor_controller[self.motor_type][2]
        self.angle_bias_1 = motor_controller[self.motor_type][0]
        self.angle_bias_2 = motor_controller[self.motor_type][1]
        self.speed_weight = motor_controller[self.motor_type][5]
        self.speed_bias_1 = motor_controller[self.motor_type][3]
        self.speed_bias_2 = motor_controller[self.motor_type][4]
        self.Angle = motor_controller[self.motor_type][6]
        self.Speed = motor_controller[self.motor_type][6]
        self.seridev = motor_controller[self.motor_type][7]
        self.auto_drive = motor_controller[self.motor_type][8]
        self.angle_offset = 0.0
        self.last_speed = 0.0
        self.g_chk_time = 0
        self.change_vector_term = 0.07

    def set_vesc(self):
        rospy.Subscriber("/sensors/core", VescStateStamped, self.VescStateCallback, queue_size=1)
        speed_max = float(rospy.get_param("/vesc_driver/speed_max"))
        speed_min = float(rospy.get_param("/vesc_driver/speed_min"))
        speed_to_erpm_gain = float(rospy.get_param("/speed_to_erpm_gain"))
        self.vesc_smax = speed_max / speed_to_erpm_gain
        self.vesc_smin = speed_min / speed_to_erpm_gain
        self.FaultCode = [
            "FAULT_CODE_NONE",
            "FAULT_CODE_OVER_VOLTAGE",
            "FAULT_CODE_UNDER_VOLTAGE",
            "FAULT_CODE_DRV8302",
            "FAULT_CODE_ABS_OVER_CURRENT",
            "FAULT_CODE_OVER_TEMP_FET",
            "FAULT_CODE_OVER_TEMP_MOTOR"
        ]

    def VescStateCallback(self, data):
        self.Battery = data.state.voltage_input
        faultcode = data.state.fault_code
        if faultcode:
            rospy.logfatal(self.FaultCode[faultcode])
        if ((time.time()-self.start_time) > 1):
            if self.debug:
                print(self.Battery)
            self.start_time = time.time()

    def ros_init(self, goal_manager, idx):
        rospy.init_node('xycar_motor', anonymous=True)

        if self.motor_type == 0:
            self.ack_msg = AckermannDriveStamped()
            self.ack_msg.header.frame_id = ''
            self.ackerm_publisher = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

        # rospy.Subscriber("cmd_vel", Twist, self.dwa_ackermann_callback, queue_size=1) # when using teb local planner
        rospy.Subscriber("cmd_vel", Twist, self.teb_ackermann_callback, queue_size=1) # when using teb local planner

        if goal_manager:
            self.gm = GoalManager(idx)

        self.angle_offset = rospy.get_param("~angle_offset")
        self.motor_type = rospy.get_param("~motor_type")


    def auto_drive_vesc(self, steer_val, car_run_speed):
        self.ack_msg.header.stamp = rospy.Time.now()
        self.ack_msg.drive.steering_angle = -steer_val
        spd = max(min(car_run_speed, self.vesc_smax), self.vesc_smin)
        cnt = 10
        min_v_piece = float(self.vesc_smax)/(2.0*float(cnt))
        if ((self.last_speed > 0) and (spd <= 0)) or ((self.last_speed < 0) and (spd >= 0)):
            #print("?", self.last_speed, spd)       
            #self.ack_msg.drive.speed = max(min(0.43 * (self.last_speed / abs(self.last_speed)), self.vesc_smax), self.vesc_smin)
            #self.ackerm_publisher.publish(self.ack_msg)
            #time.sleep(self.change_vector_term)
            reduce_v = max(abs(float(spd) - float(self.last_speed)) / float(cnt), min_v_piece)
            vector = -(self.last_speed / abs(self.last_speed))
            for c in range(cnt):
                s = float(self.last_speed) + (vector * c * reduce_v)
                if vector > 0:
                    self.ack_msg.drive.speed = min(s, float(spd))
                elif vector < 0:
                    self.ack_msg.drive.speed = max(s, float(spd))
                self.ackerm_publisher.publish(self.ack_msg)
                time.sleep(self.change_vector_term)
            time.sleep(self.change_vector_term)
        if self.last_speed == 0:
            if float(spd) < 0:
                reduce_v = min(float(spd) / float(cnt), min_v_piece)
            elif float(spd) > 0:
                reduce_v = max(float(spd) / float(cnt), min_v_piece)
            else:
                cnt = 0
            for c in range(cnt):
                s = float(self.last_speed) + (reduce_v * c)
                if float(spd) < 0:
                    self.ack_msg.drive.speed = max(s, float(spd))
                elif float(spd) > 0:
                    self.ack_msg.drive.speed = min(s, float(spd))
                #print("hs", self.ack_msg.drive.speed)
                self.ackerm_publisher.publish(self.ack_msg)
                time.sleep(self.change_vector_term)
            time.sleep(self.change_vector_term)

        self.ack_msg.drive.speed = float(spd)
        #print("ra, rs", -steer_val, spd)
        self.ackerm_publisher.publish(self.ack_msg)
        self.last_speed = float(spd)
    

    def teb_ackermann_callback(self, msg):
        speed = msg.linear.x
        steering_angle = -msg.angular.z
                
        if self.gm is not None:
            if self.gm.stop_node.detected and self.stop_cnt == 0 and self.gm.goal_list[self.gm.goal_cnt].stop_flag:
                speed = 0
                self.auto_drive(steering_angle, speed)
                rospy.sleep(3.)
                self.stop_cnt += 1

            else:
                self.auto_drive(steering_angle, speed)
        
        else:
            self.auto_drive(steering_angle, speed)


if __name__ == '__main__':
    m = motor(sys.argv[1], int(sys.argv[2]))
