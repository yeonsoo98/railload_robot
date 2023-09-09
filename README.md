# Railload Robot

## 현재 진행 사항
- Modbus (RS485) 통신 연결 < 모터드라이버 > - < 잿슨 오린 >
  - driving.py
    
- Keyboard & Joystick 제어 가능
  - keyboard_comand.py
  - joystick_command.py
    
- Person Tracker
  - tracker.py
  - YOLOv5 기반의 사람 추종
  - ( 1.5 m < Distance ) => Stop & ( 1.5 m > Distance ) => Go
 
    
- Simulation
  - description 확인 가능
  - body , wheel xacro file로 구성 진행중
  - gazebo & rqt 

## 향후 계획
- 세부적인 rpm 설정하기 & 제어하기
- Slam & Navigation
- GPS 
- 시뮬레이션 환경 구축하기

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


## Railload Robot Explanation
* Functionality
  * driving.py: motor_protocol & start_node
  
  
  * teleop.keyboard: motor_control
    - teleop keyboard input -> linear.x > 0.0 -> 1000 rpm
    - teleop keyboard input -> linear.x < 0.0 -> - 1000 rpm
    
    
    
  * keyboard_command.py: motor_speed_control
    - q : 300 rpm
    - w : 500 rpm
    - e : 700 rpm
    - r : 1000 rpm
    - z : - 1000 rpm
    - s : 0 rpm (Stop)
    
    
    
  * Joystick control
    - ros2 run joy joy_node 
    - ros2 run teleop_twist_joy teleop_node
    - driving.py
    - Code start / Joystick -  Push R1 & Wheel

  * Person Tracker
      - launch ros_camera
      - YOLOv5 model 
      - object detect -> person / ... 
      - driving.py 
      - tracker.py
      
            - use depth camera (realsense2) to check object distance
            - person distance < 1.5 m   -> robot stop 
            - person distance > 1.5 m   -> robot go 
            - restict / camera performance - 10 m
            - closet distance 
            - count people
            - motor delay (with timer)

* Hardware settings 
SBC -> Motor Drive (USE RS 485) Modbus Protocol -> Motor 

  * SBC ( Nvidia Jetson Orin )
![image](https://user-images.githubusercontent.com/77741178/224618275-10c6d570-fa22-433d-9831-7383f32426c2.png)

   * Power Supply Device
![image](https://user-images.githubusercontent.com/77741178/224618244-44db1496-8302-4cf3-af5c-9057796d6f1b.png)

   * Motor
![image](https://user-images.githubusercontent.com/77741178/224618255-6d390bf3-1917-4afb-940e-30888b9c135a.png)   

   * Motor Drive
![image](https://user-images.githubusercontent.com/77741178/224618265-b069b8f8-3a3d-48df-b604-611ab070acca.png)

   * Joystick
![조이스틱](https://user-images.githubusercontent.com/77741178/227871724-d1294560-3b27-4d87-95a4-6c7b3c2d1ae8.jpeg)


- TBD ...Railload
