# Javis

광운대학교 로봇학부 학술소모임 **'BARAM'** 20년도 후반기 **'Javis'** Project에 대한 소스코드입니다.  

## 개발 환경
|OS|사용 언어|사용 IDE|
|:---:|:---:|:---:|
| ROS-kinetic( Ubuntu 16.04 ) | C++ | Qt creater |

## 프로젝트 개발 동기

-  몸이 불편하신 분들을 옆에서 보조하면서 도울 수 있는 6-DOF manipulator를 제작하였으며, SLAM과  Navigation 을 활용하여 자율 주행이 가능한 mobile manipulator인 Javis를 제작하였습니다.

## 프로젝트 개요
1. Fusion Slam(2d gmapping slam, 3D rtabmap slam)
2. Localization & Navigation
3. EKF(Odometry Extended Kalman Filter)
4. Manipulator(Jacobian)
5. Mobile Robot(Pioneer 3dx), Motor(Dynamixel EX-106, MX-106, RX-64, AX-12+)

## System Architecture
<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103438973-25470880-4c7c-11eb-8802-9935402edea9.jpg" width="600px"></p>  


### Code Overview  
- Javis_pkg, Javis_arm : 프로젝트 main  package
- SLAM(Gmapping, Rtabmap_ros)
- 그 외의 구동에 필요한 package
> 1. (amr-ros-config : RVIZ에서 Pioneer 3dx를 시각화 하기 위한 package)
> 2. (DynamixelSDK : Manipulator에 들어가는 dynamixel들을 구동하기 위한 package)
> 3. (freenect_stack : Kinect v1을 구동하기 위한 package)
> 4. (rosaria : Pioneer 3dx 모델을 ROS로 구동하기 위한 package)
> 5. (robot_localization : EKF를 사용하기 위한 package)
> 6. (teleop_keyboard : 키보드 입력으로 Pioneer 3dx를 조종하기 위한 package)


### Project scenario

1. ROS가 실행되며 Gmapping, Rtabmap_ros를 통해 각각 2D, 3D map을 그리며 저장한다. 또한 Odometry error는 robot_localizatoin를 통해 보정된다.
2. Rtabmap_ros를 통해 그려진 map을 기준으로 주변 물체의 좌표 값 추출(Depth data)
3. 해당 좌표 근처까지 Localization, Navigation을 통해 이동한다.
4. Manipulator
> - 해당 좌표의 물체를 Jacobian 계산을 통해 물체를 집고 다음 navigation point로 이동한 후 2), 3)번부터 다시 반복한다.
> - 해당 좌표의 물체를 Jacobian 계산을 통해 물체를 집고 고정하는 등 사용자에게 보조 역할을 제공한다.


## 프로젝트 결과

<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103438929-d00af700-4c7b-11eb-9379-527b85f5c9bc.jpg" width="500px"></p>  
<p align="center">  프로젝트 완성작 모습 </p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103438931-d7ca9b80-4c7b-11eb-9245-2ec4dbce233c.gif" width="500px"></p>  
<p align="center"> Rtapmap + Gmapping 동시 SLAM 구현 모습 </p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103438932-d8fbc880-4c7b-11eb-8d50-19552cc45042.gif" width="500px"></p>  
<p align="center"> Manipulator 구동모습 </p>  

