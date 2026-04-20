# kcare_robot_ros2_sdk
모바일 매니퓰레이터 로봇 제어부 ROS2 SDK(ROS2 Humble)

(URDF 파일은 kcare_description에서 확인 가능)


## 1. Preparation

* Ubuntu 22.04 + ROS2 Humble 혹은 도커 컨테이너
    - [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

## 2. 패키지 다운로드 및 설치
- 프로젝트 레포지토리 다운로드
    ```bash
    git clone https://github.com/keti-ai/kcare_robotics_sdk_ros2.git
    ```
- 서브모듈 다운로드.(Xarm ros2, Orbbec Camera ROS2)
    ```bash
    cd kcare_robotics_sdk_ros2
    git submodule sync
    git submodule update --init --remote --recursive --progress
    ```

## 3. Orbbec Camera 의존성 패키지 설치 및 빌드
- #### 3.1 Orbbec Camera 의존성 패키지 설치
    ```bash
    # assume you have sourced ROS environment, same blow
    sudo apt install libgflags-dev nlohmann-json3-dev  \
    ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
    ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
    ros-$ROS_DISTRO-backward-ros libdw-dev
    ```
- #### 3.2 Install udev rules
    ```bash
    # 레포지토리내 submodule 경로로 이동
    cd  src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
    sudo bash install_udev_rules.sh
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
- #### 3.3 ROS2 패키지 빌드
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    # Orbbec SDK 패키지만 설치.(다른 패키지는 Release로 설치되면 안되서 분리해서 설치)
    colcon build --packages-select orbbec_camera orbbec_camera_msgs orbbec_description --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
    ```

## 4. Xarm ROS2 패키지 의존성 패키지 설치 및 빌드
- #### 4.1 레포지토리 업데이트.(submodule update를 완료했으면 필요 X)
    ```bash
    cd  src/third_party/xarm_ros2
    git pull
    git submodule sync
    git submodule update --init --remote
    ```

- #### 4.2 의존성 패키지 설치
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src/third_party/xarm_ros2 --ignore-src --rosdistro $ROS_DISTRO -y
    ```
- #### 4.3 Config 파일 수정
    Tool Modbus 활성화를 위해 xarm_ros2/xarm_api/config/xarm_params.yaml 파일에서 아래 부분 수정
    ```yaml
    set_tool_position: true
    ```

- #### 4.4 Xarm ROS2 패키지 빌드
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    # build selected packages
    colcon build --packages-up-to xarm_api xarm_msgs uf_ros_lib
    ```

## 5. Kcare robotics 프로그램 셋업
- #### 5.1 파이썬 필요 패키지 설치
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    pip install -r requirements.txt
    ```

- #### 5.2 패키지 빌드(기존 빌드된 ROS2 패키지 소스 필요)
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    # xarm, orbbec 패키지 소스
    source install/setup.bash
    # 파이썬 패키지 심볼릭 링크 빌드
    colcon build --symlink-install --packages-select kcare_robot_ros2_controller kcare_robot_ros2_controller_msgs kcare_description
    ```



## 6 테스트 및 실행


- #### 6.1 ROS2 통신을 위한 네트워크 버퍼 크기 조절
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    bash setup_network_buffer.sh
    ```

- #### 6.2 UDEV RULES로 시리얼 포트 이름 고정
    - 연결된 USb 장치 확인 및 시리얼 고유번호 확인
    ```bash
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
    ```

    - 시리얼 포트 심볼릭 링크 생성
    ```bash
    sudo nano /etc/udev/rules.d/99-serial.rules
    ```

    - 아래와 같이 작성. U2D2는 ttyHead, 다른 USBto485 장치는 ttyLM으로, 그리퍼는 ttyGripper로 
    ```
    SUBSYSTEM=="tty", ATTRS{serial}=="B002GC9X", SYMLINK+="ttyLM", ATTR{latency_timer}="1"
    SUBSYSTEM=="tty", ATTRS{serial}=="FT66WNSB", SYMLINK+="ttyHead", ATTR{latency_timer}="1"
    SUBSYSTEM=="tty", ATTRS{serial}=="BG01N7QK", SYMLINK+="ttyGripper", ATTR{latency_timer}="1"
    ```

    - Udev룰 재적용. 재적용후 usb 장치 재연결 필요 (필요시 sudo reboot로 재부팅)
    ```bash
    sudo service udev reload
    sudo service udev restart
    ```

    - 아래 명령어로 확인
    ```bash
    # 연결된 USB 포트 목록 확인
    ls /dev/tty*

    # /dev/ttyHead
    # /dev/ttyLM
    ```

    - Dynamixel Wizard 세팅
    * Dynamixel Wizard Download
        - [Dynamixel EManual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
    - 시리얼 포트 권한 관련 설정
    ```bash
    sudo usermod -aG dialout $USER
     # 필요시 실행
    sudo reboot
    ```

    - Setting에서 아래와 같이 설정후 Search
        - Protocol -> 2.0
        - Port -> Dynamixel 연결 포트
        - Baud -> 57600 (만약안되면 115200)
        - ID -> 0~2 (ID를 넓게 하면 시간 많이 소요)

    - Scan으로 Dynamxiel 연결 및 파라미터 수정
        - Drive Mode -> Bit 0 Reverse Mode 체크 후 저장(ID 1, 2 둘다 변경)
        - Baud Rate -> 115200 (하나만 바꿔도 전체 적용)


- #### 6.2 Xarm Ros2 Node 실행
    ```bash
    source install/setup.bash
    ros2 launch xarm_api xarm7_driver.launch.py \
        robot_ip:=192.168.5.212 \   # 로봇 IP에 맞게 IP 변경경
    ```

- #### 6.3 Orbbec Camera Ros2 Node 실행
    - 연결된 카메라 USB 포트 번호 확인
    ```bash
    # 레포지토리내 submodule 경로로 이동
    cd src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
    ./list_ob_devices.sh
    ```
    - Femto Bolt 카메라 실행 노드.
    ```bash
    # 레포지토리 루트 디렉토리로 이동
    cd kcare_robotics_sdk_ros2
    source install/setup.bash
    # Femto Bolt Camera 노드 실행
    ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=femto \
        depth_registration:=true \
        color_qos:=SENSOR_DATA \
        depth_qos:=SENSOR_DATA
    ```


    - #### 6.4 Kcare robot 로봇노드 실행(Xarm Node와 동시 실행)
        - kcare_robot_ros2_controller/launch/nsquare_robot.launch.py에서 아래부분 IP 변경
        ```python
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
        ```

