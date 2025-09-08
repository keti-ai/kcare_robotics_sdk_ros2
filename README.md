# kcare_robot_ros2_sdk
모바일 매니퓰레이터 로봇 제어부 ROS2 SDK(ROS2 Humble)

(URDF 파일은 kcare_description에서 확인 가능)


## 1. Preparation

* Ubuntu 22.04 + ROS2 Humble 혹은 도커 컨테이너
    - [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

* Xarm ROS2 소스 패키지(humble branch 사용)

    - [xarm-ros2](https://github.com/xArm-Developer/xarm_ros2/tree/humble)

* Orbbec Camera ROS2 SDK 소스 패키지(v2-main branch 사용)

    - [OrbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)

## 2. 작업공간 생성 및 소스코드 복사

### 2.1 워크스페이스 생성

    # 홈 디렉토리 혹은 작업공간 디렉토리로 이동
    & cd ~
    # 3개의 workspace 생성(빌드시 작업공간 분리를 위해 사용)
    & mkdir -p kcare_ws/src camera_ws/src xarm_ws/src


### 2.2 각각의 소스코드를 각 workspace에 복사

    # Kcare 모바일로봇 소스코드 복사
    & cd ~/kcare_ws/src
    & git clone https://github.com/keti-ai/kcare_robotics_sdk_ros2.git
    
    # Orbbec Ros2 SDK 소스코드 복사
    & cd ~/camera_ws/src
    & git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
    & cd OrbbecSDK_ROS2
    & git checkout v2-main

    # Xarm ROS2 SDK 소스코드 복사
    & cd ~/xarm_ws/src
    & git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO

## 3 의존성 패키지 설치 및 빌드
### 3.1 Orbbec Camera 의존성 패키지 설치 및 빌드
#### 3.1.1 의존성 패키지 설치 및 빌드

    # assume you have sourced ROS environment, same blow
    sudo apt install libgflags-dev nlohmann-json3-dev  \
    ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
    ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
    ros-$ROS_DISTRO-backward-ros libdw-dev

#### 3.1.2 Install udev rules.

    cd  ~/camera_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
    sudo bash install_udev_rules.sh
    sudo udevadm control --reload-rules && sudo udevadm trigger

#### 3.1.3 패키지 빌드

    cd ~/camera_ws/
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release


### 3.2 Xarm ROS2 패키지 의존성 패키지 설치 및 빌드
#### 3.2.1 레포지토리 업데이트

    cd ~/xarm_ws/src/xarm_ros2
    git pull
    git submodule sync
    git submodule update --init --remote

#### 3.2.2 의존성 패키지 설치

    cd ~/xarm_ws/src
    rosdep update
    rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

### 3.2.3 config 파일 수정

Tool Modbus 활성화를 위해 xarm_ros2/xarm_api/config/xarm_params.yaml 파일에서 아래 부분 수정

    set_tool_position: false --> true
    set_tgpio_modbus_timeout: false --> true
    get_tgpio_modbus_baudrate: false --> true
    set_tgpio_modbus_baudrate: false --> true
    getset_tgpio_modbus_data: false --> true

#### 3.2.4 빌드

    cd ~/xarm_ws/
    # build all packages
    colcon build

    # build selected packages
    colcon build --packages-select xarm_api xarm_msgs


### 3.3 Kcare robotics 프로그램 셋업

#### 3.3.1 파이썬 필요 패키지 설치

    cd ~/kcare_ws/src/kcare_robotics_sdk_ros2/kcare_robot_ros2_controller/
    pip install -r requirements.txt

#### 3.3.2 패키지 빌드

    cd ~/kcare_ws/
    colcon build --symlink-install


## 4 테스트 및 실행

### 4.1 Xarm Ros2 Node 실행

    source ~/xarm_ws/install/setup.bash
    ros2 launch xarm_api xarm7_driver.launch.py \
        robot_ip:=192.168.1.245 \   # 로봇 IP에 맞게 IP 변경경
        add_gripper:=true

### 4.2 Orbbec Camera Ros2 Node 실행

#### Femto Bolt 카메라 실행 노드. 시리얼 넘버 변경 확인

    source ~/camera_ws/install/setup.bash
    ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=femto \
        depth_registration:=true \
        enable_point_cloud:=true\
        serial_number:=CL8K14100X2 \
        color_fps:=15 \
        depth_fps:=15

#### Gemini2 카메라 실행 노드. 시리얼 넘버 변경 확인. 

    source ~/camera_ws/install/setup.bash
    ros2 launch orbbec_camera gemini2.launch.py camera_name:=hand \
        depth_registration:=true \
        device_num:=2 \ ##카메라가 여러개 연결되어있을경우 카메라 장치 순서 확인후 파라미터 추가
        serial_number:=AY3N3310006 \
        color_fps:=15 \
        depth_fps:=15 \
        depth_work_mode:="Unbinned Dense Default"

### 4.3 Kcare robot 로봇노드 실행(Xarm Node와 동시 실행)

#### 4.3.1 kcare_robot_ros2_controller/launch/kcare_robot.launch.py에서 아래부분 IP 변경

    robot_ip_arg = LaunchConfiguration('robot_ip',default='192.168.1.245')

    return LaunchDescription([
        # Robot IP 설정 인자
        DeclareLaunchArgument('robot_ip',default_value='192.168.1.245',description='IP address of the robot'),

#### 4.3.2 로봇 제어노드 실행

    source ~/camera_ws/install/setup.bash
    source ~/xarm_ws/install/setup.bash
    source ~/kcare_ws/install/setup.bash

    ros2 launch kcare_robot_ros2_controller kcare_robot.launch.py
