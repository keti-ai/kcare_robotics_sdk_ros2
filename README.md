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
    & git clone -b nsquare https://github.com/keti-ai/kcare_robotics_sdk_ros2.git
    
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
    # build selected packages
    colcon build --packages-up-to xarm_api xarm_msgs uf_ros_lib


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
        robot_ip:=192.168.5.212 \   # 로봇 IP에 맞게 IP 변경경

### 4.2 Orbbec Camera Ros2 Node 실행

#### 연결된 카메라 USB 포트 번호 확인

    cd ~/camera_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/
    ./list_ob_devices.sh

    # 결과는 아래와 같이 출력. 각 장치의 USB Port 넘버에 대응하는 번호를 카메라 런치 입력인자로 전달
    #Found Orbbec device Orbbec(R) Gemini(TM), usb port 1-4.4, serial number AY3794301NF
    #Found Orbbec device Orbbec Femto Bolt 3D Camera, usb port 2-3.1, serial number CL83842008X


#### Femto Bolt 카메라 실행 노드. 시리얼 넘버 변경 확인

    source ~/camera_ws/install/setup.bash
    ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=femto \
        depth_registration:=true \
        device_num:=2 \
        usb_port:=2-3.1 \
        color_qos:=SENSOR_DATA \
        depth_qos:=SENSOR_DATA

#### Gemini2 카메라 실행 노드. 시리얼 넘버 변경 확인. 

    source ~/camera_ws/install/setup.bash
    ros2 launch orbbec_camera gemini2.launch.py camera_name:=hand \
        depth_registration:=true \
        device_num:=2 \
        usb_port:=1-4.4 \
        color_qos:=SENSOR_DATA \
        depth_qos:=SENSOR_DATA

#### Femto Bolt, Gemini2 카메라 동시 실행

- OrbbecSDK_ROS2/orbbec_camera/launch/multi_camera.launch.py 코드 아래와 같이 수정

```python
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    import os

    def generate_launch_description():
        # Include launch files
        package_dir = get_package_share_directory('orbbec_camera')
        launch_file_dir = os.path.join(package_dir, 'launch')
        launch1_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'femto_bolt.launch.py')
            ),
            launch_arguments={
                'camera_name': 'femto',
                'depth_registration': 'true',
                'usb_port': '2-3.1',    # 실제 USB 포트에 맞게 수정
                'device_num': '2',
                'color_qos': 'SENSOR_DATA',
                'depth_qos': 'SENSOR_DATA',
            }.items()
        )

        launch2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'gemini2.launch.py')
            ),
            launch_arguments={
                'camera_name': 'hand',
                'depth_registration': 'true',
                'usb_port': '1-4.4',    # 실제 USB 포트에 맞게 수정
                'device_num': '2',
                'color_qos': 'SENSOR_DATA',
                'depth_qos': 'SENSOR_DATA',
            }.items()
        )

        # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

        # Launch description
        ld = LaunchDescription([
                TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
                TimerAction(period=2.0, actions=[GroupAction([launch1_include])]),
        ])

        return ld
```

- 코드 수정내용 반영을 위해 빌드
```bash
    cd ~/camera_ws/
    colcon build --event-handlers  console_direct+  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

####  ROS2 통신을 위한 네트워크 버퍼 크기 조절

    cd ~/kcare_ws/src/kcare_robotics_sdk_ros2/scripts
    bash setup_network_buffer.sh

#### UDEV RULES로 시리얼 포트 이름 고정
##### 연결된 USb 장치 확인 및 시리얼 고유번호 확인

    # 연결된 USB 포트 목록 확인
    ls /dev/tty*

    # 아래와 같은 결과 나오는지 확인
    # ...
    # /dev/ttyUSB0
    # /dev/ttyUSB1
    # ...

    # 아래 명령어 실행으로 시리얼 포트 고유 시리얼넘버 확인
    udevadm info -a /dev/ttyUSB0 | grep '{serial}'
    udevadm info -a /dev/ttyUSB1 | grep '{serial}'

    # 아래와 같은 결과 출력
    #ATTRS{serial}=="FTA7NMDN"    # 각 장치별 이 값을 메모
    #ATTRS{serial}=="3610000.usb"

##### 시리얼 포트 심볼릭 링크 생성

    cd /etc/udev/rules.d
    sudo nano 99-tty.rules

##### 아래와 같이 작성. U2D2는 ttyHead, 다른 USBto485 장치는 ttyLM으로 

    SUBSYSTEM=="tty", ATTRS{serial}=="FTA7NMDN", SYMLINK+="ttyHead"
    SUBSYSTEM=="tty", ATTRS{serial}=="BG01LYXW", SYMLINK+="ttyLM"
    
##### Udev룰 재적용. 재적용후 usb 장치 재연결 필요 (필요시 sudo reboot로 재부팅)

    sudo service udev reload
    sudo service udev restart

##### 아래 명령어로 확인

    # 연결된 USB 포트 목록 확인
    ls /dev/tty*

    # /dev/ttyHead
    # /dev/ttyLM


#### Dynamixel Wizard 세팅

* Dynamixel Wizard Download
    - [Dynamixel EManual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

##### 시리얼 포트 권한 관련 설정

    sudo usermod -aG dialout $USER
     # 필요시 실행
    sudo reboot

##### Setting에서 아래와 같이 설정후 Search
- Protocol -> 2.0
- Port -> Dynamixel 연결 포트
- Baud -> 57600 (만약안되면 115200)
- ID -> 0~2 (ID를 넓게 하면 시간 많이 소요)


##### Scan으로 Dynamxiel 연결 및 파라미터 수정
- Drive Mode -> Bit 0 Reverse Mode 체크 후 저장(ID 1, 2 둘다 변경)
- Baud Rate -> 115200 (하나만 바꿔도 전체 적용)


### 4.3 Kcare robot 로봇노드 실행(Xarm Node와 동시 실행)

#### 4.3.1 kcare_robot_ros2_controller/launch/nsquare_robot.launch.py에서 아래부분 IP 변경

    def generate_launch_description():
        xarm_ip_value = '192.168.5.212'    # 이부분 IP 로봇에 맞게 변경
        # Include launch files
        xarm_package_dir = get_package_share_directory('xarm_api')
        xarm_launch_file_dir = os.path.join(xarm_package_dir, 'launch')
        xarm_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xarm_launch_file_dir, 'xarm7_driver.launch.py')
            ),
            launch_arguments={
                'robot_ip': xarm_ip_value, # JSON에서 로드된 IP를 사용
            }.items()
        )

#### 4.3.2 로봇 제어노드 실행

    source ~/camera_ws/install/setup.bash
    source ~/xarm_ws/install/setup.bash
    source ~/kcare_ws/install/setup.bash

    # 필요시 alias 명령어로 등록
    ros2 launch kcare_robot_ros2_controller nsqare_robot.launch.py

    # 멀티 카메라 비전 노드 실행
    ros2 launch orbbec_camera multi_camera.launch.py
