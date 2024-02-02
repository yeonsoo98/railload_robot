# RailAMR
* **RailAMR 개발** - 궤도를 자율 주행하는 로봇으로 작업자의 안전 관리와 작업 지원을 통한 
작업자 안전 강화, 철도 시설 검층 장치 부착을 통한 자동점검 기능 수행 목표
  - YOLOv5와 Depth Camera를 통한 사람 추종 주행 & 안전 주행
  - 작업자가 손쉽게 운용할 수 있도록 키보드 & 조이스틱 주행
  - 안전한 주행 테스트를 위한 Gazebo 시뮬레이션 환경 개발
  - 자율 주행을 위한 SLAM & Naviagition 기능 탑재


- RailAMR
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/53797ddc-74d1-4ee8-af5d-0d1170f7198c" width="400" height="200">

- RailAMR System Architecture
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/31956012-cecc-4c6e-b4da-c2e6770159d0" width="400" height="200">

- RailAMR Configuration diagram
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/8b5c6f9b-8a1c-41a9-9145-8ef78f15772a" width="400" height="200">


## 기능 정리
- Modbus (RS485) 통신 연결 ( 모터드라이버 < - > 잿슨 오린 )
  - driving.py (로봇의 통신을 담당하는 코드 - 로봇을 작동시킬시, 항상 켜져있어야 함)  
```
ros2 launch railload_robot_bringup driving.py
```
    
- Keyboard & Joystick 제어 기능
  - keyboard_comand.py (블루투스 및 USB로 연결된 키보드로 조종 가능)
```
ros2 launch railload_robot_bringup keyboard_command.py
```

  * keyboard_command.py: motor_speed_control (해당 키를 누르면 해당 rpm 속도로 이동)
    - q : 300 rpm
    - w : 500 rpm
    - e : 700 rpm
    - r : 1000 rpm
    - z : - 1000 rpm
    - s : 0 rpm (Stop)
      
  - joystick_command.py (블루투스로 연결된 조이스틱으로 조종 가능)
```
ros2 launch railload_robot_bringup joystick_command.py
```

  * Joystick control
    - ros2 run joy joy_node (조이스틱 블루투스 연결)
    - ros2 launch railload_robot_bringup joystick_command.py (조이스틱 실행)
    - driving.py (로봇 통신 연결)
    - Code start / Joystick -  Push R1 & Wheel (조이스틱 R1키를 동시에 누르기)

- Person Tracker (사람 추종 및 안전 주행 기능)
  - YOLOv5 기반의 사람 추종
    - Depth Camera를 통한 사람과의 거리 측정
    - ( 1.5 m < Distance ) => Stop 
    - ( 1.5 m > Distance ) => Go
    - 사람의 안전거리를 (1.5m) 파악하여 자율 주행
    - autostart를 통해 잭슨 오린 실행시 자동으로 시작

  - yolo.launch.py
``` 
ros2 launch railload_robot_bringup yolo.launch.py
```
      
  - tracker.py
```
ros2 launch railload_robot_bringup tracker.py
```

  * Person Tracker

            - use depth camera (realsense2) to check object distance
            - person distance < 1.5 m   -> robot stop 
            - person distance > 1.5 m   -> robot go 
            - restict / camera performance - 10 m
            - closet distance 
            - count people
            - motor delay (with timer)

  - robot.launch.py (yolo + tracker 동시 실행하는 코드)
```
ros2 launch railload_robot_bringup robot.launch.py
```

* 로봇 센서 데이터 및 운용 정보 모니터링 화면
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/2f7f5164-3e0e-4135-84a2-eea9a8b130f3" width="400" height="200">

* 로봇 주행 기능
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/79ad117b-5308-438d-9687-58d38f09366a" width="400" height="200">


- Robot Simulation
  - 기존 로봇 제어 코드로 제어 가능
  - description 확인 가능
  - body , wheel xacro file로 구성 
  - ROS2 기반 Velodyne-VLP 16 LiDAR 연동
  - gazebo & rqt & rviz2

  - display.launch.py (gazebo + rviz2 동시 실행)
```
ros2 launch railload_robot_description display.launch.py
```

  - gazebo.launch.py (gazebo 실행)
```
ros2 launch railload_robot_description gazebo.launch.py
```

  - Lidar.py (velodyne vlp-16 lidar 실행)
```
ros2 launch railload_robot_description Lidar.py
```

* Gazebo & Rviz2 실행
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/3abfb8c7-5aaf-41d8-b07e-c875b5ecc237" width="400" height="200">

* LiDAR & GPS 센서 데이터 수집
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/b539f8bd-ab40-4415-953d-30a09f2b480b" width="400" height="200">

## 추가 기능
- 세부적인 rpm 설정하기 & 제어하기
```
ros2 launch railload_robot_description rpm_calculate.py
```

## Railload Robot Driving with sensor data 
- https://github.com/yeonsoo98/railload_robot/issues/6

## Railload Robot Tracker Driving 
- https://github.com/yeonsoo98/railload_robot/issues/5

## Railload Robot Safety Driving
- https://github.com/yeonsoo98/railload_robot/issues/4

## Railload Robot Video 
- https://github.com/yeonsoo98/railload_robot/issues/1 

## Motor Control Video 1
- https://github.com/yeonsoo98/railload_robot/issues/2

## Motor Control Video 2
- https://github.com/yeonsoo98/railload_robot/issues/3

* Hardware settings 
  * SBC ( Nvidia Jetson Orin )
<img src="https://user-images.githubusercontent.com/77741178/224618275-10c6d570-fa22-433d-9831-7383f32426c2.png" width="400" height="200">

   * Motor
<img src="https://user-images.githubusercontent.com/77741178/224618255-6d390bf3-1917-4afb-940e-30888b9c135a.png" width="400" height="200">

   * Motor Drive
<img src="https://user-images.githubusercontent.com/77741178/224618265-b069b8f8-3a3d-48df-b604-611ab070acca.png" width="400" height="200">

   * Joystick
<img src="https://user-images.githubusercontent.com/77741178/227871724-d1294560-3b27-4d87-95a4-6c7b3c2d1ae8.jpeg" width="400" height="200">

   * Velodyne VLP-16 & Intel Realsense Depth Camera
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/b94e0a8b-010c-44a6-8215-4a17b6917222" width="400" height="200">

   * Driving Environment
<img src="https://github.com/yeonsoo98/railload_robot/assets/77741178/433c0736-4817-49e7-af08-834a36e47c70" width="400" height="200">


- TBD ...RailAMR
