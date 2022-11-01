# Hell-Coding

Repository of Hell-Coding Team for 2022 KMU autonomous driving competition 

Teammate: 정종현, 하지훈, 양승진, 이현민, 황석민

---
## Environment

- OS: Ubuntu 18.04 
- Board: Nvidia TX2
- Motor: Vesc Driver
- LiDAR: RPLiDAR A2
- CAM: ELP 180deg, Fisheye Lens 1080p Webcam
- IMU: MPU-9250
- Ultrasonic Sensor: HC-SR04 x 5


---
## Dependency

* ROS Navigation Stack
* Vesc Driver
* Image Proc
* AR Marker
* Numpy
* Pandas

For optional
+ ORB SLAM2

---

## Usage

For mapping,

    roslaunch hell_coding hell_mapping.launch


For navigation,

    roslaunch hell_coding hell_navigation.launch
    
