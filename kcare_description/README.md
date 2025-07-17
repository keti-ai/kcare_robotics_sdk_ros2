# kcare_description

모바일 매니퓰레이터 URDF 패키지.(Gripper Update Requires)


## 1. Preparation


* Xarm ROS2 패키지 사전 빌드. 패키지 내 xarm_description 이 빌드되어 있어야 합니다.

    - [xarm-ros2](https://github.com/xArm-Developer/xarm_ros2/tree/humble)


## 2. Xarm ros2 패키지 빌드

    # Xarm ROS2 SDK 소스코드 복사
    & mkdir -p ~/xarm_ws/src
    & cd ~/xarm_ws/src
    & git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
    & cd ~/xarm_ws
    & colcon build --packages-select xarm_description

## 3. kcare_description 패키지 빌드

    # Xarm ROS2 SDK 소스추가
    & source ~/xarm_ws/install/setup.bash
    # kcare_description 패키지 작업 디렉토리 생성후 소스 복사, 빌드
    & mkdir -p ~/kcare_ws/src
    & cd ~/kcare_ws/src
    & git clone https://github.com/keti-ai/kcare_robotics_sdk_ros2.git
    & colcon build --packages-select kcare_description


## 4. Test URDF Packages

### 4.1 Visualize Robots

    source ~/xarm_ws/install/setup.bash
    source ~/kcare_ws/install/setup.bash
    ros2 launch kcarebot_description robot_display.launch.py

## 5. URDF to Xacro

    cd ~/kcare_ws/src/kcare_robotics_sdk_ros2/kcare_description/robots/

    ros2 run xacro xacro kcare_platform.urdf.xacro > kcare_platform.urdf