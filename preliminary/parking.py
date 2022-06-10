#! /usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, 
          "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()

#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )

#=========================================
# 메인 제어를 위한 변수 설정
#=========================================

# P 제어를 위한 gain 설정
kp_yaw = 1.8        # yaw gain
kp_theta = 3        # theta gain

# D 제어를 위한 gain 설정
kd_yaw = 3          # yaw gain
kd_theta = 3        # theta gain

# D 제어에서 미분값을 계산하기 위한 변수
prev_time = 0       # 시간
prev_yaw = 0        # yaw
prev_theta = 0      # theta

rate = rospy.Rate(120)


#=========================================
# 메인 루프 
# 끊임없이 루프를 돌면서 
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
# 작업을 반복적으로 수행함.
#=========================================

while not rospy.is_shutdown():
    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((100, 500, 3))

    # 4개의 직선 그리기
    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    # DX값에 해당하는 위치에 동그라미 그리기 
    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1)) 

    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

    #=============================================
    # 조향각과 속도를 제어하기 위한 부분.            
    # AR 태그까지의 거리 정보와 AR 태그에 대한        
    # 상대적인 yaw 값을 제어에 활용                  
    #=============================================

    '''핸들 조향각 anlgle 값 설정하기'''

    # 시간에 대한 변화량을 구하기 위해서는 현재 시간이 필요
    cur_time = time.time()     

    # DX, DY 값으로 구한 AR 태그까지의 상대적인 각도를 theta라고 함.
    theta = math.degrees(math.atan2(int(arData["DX"]), int(arData["DY"])))

    diff_theta = (theta - prev_theta) / (cur_time - prev_time)      # D 제어를 위해 theta의 시간에 대한 변화량 산출
    diff_yaw = (yaw - prev_yaw) / (cur_time - prev_time)            # D 제어를 위해 yaw의 시간에 대한 변화량 산출

    # P, D 제어를 사용하여 차량의 조향각(angle)을 계산
    angle = kp_yaw * (-yaw) + kd_yaw*diff_yaw + kp_theta * theta + diff_theta * kd_theta

    # 최대 우측 조향각이 50 이므로, 이보다 크면 50으로 고정
    if angle > 50:
        angle = 50

    # 최대 좌측 조향각이 -50 이므로, 이보다 작으면 -50으로 고정
    elif angle < -50:
        angle = -50

    # 디버깅을 위해 theta, yaw, angle, distance 값 표시
    print("theta: "+str(theta)+" yaw: "+str(yaw)+" final angle: "+str(angle)+" distance: "+str(distance))
            
    '''차량의 속도 speed 값 설정하기'''
    speed = 10
    
    # distance가 가까워질수록 속도를 낮춰 주차함.
    if distance < 100 and distance > 80:
        speed = 5
    elif distance < 80 and distance > 65:
        speed = 1
    elif distance < 65:
        speed = 0
    
    # 카메라 각도에 AR 태그 전체부분이 들어오지 않으면 distance를 인식하지 못하다가
    # 가까이 다가가면 다시 인식하는 모습을 보임.
    # 다시 distance가 인식되는 가까운 거리는 주차구역을 벗어난 곳이므로 후진하여 자세 조정
    if distance < 50:
        speed = -2


    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)

    # D 제어를 위해 이전 값 기억
    prev_time = cur_time        # 시간에 대한 변화량을 구하기 위해 cur_time의 이전 값 기억
    prev_yaw = yaw              # 시간에 대한 변화량을 구하기 위해 yaw의 이전 값 기억
    prev_theta = theta          # 시간에 대한 변화량을 구하기 위해 theta의 이전 값 기억

    rate.sleep()    


# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()





