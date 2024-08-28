# 환경 설정

## 실습 환경

- PC
    - VirtualBox VM
    - Ubuntu 20.04(Focal Fossa)
- Turtlebot 3 Burger
    - 2022년 개선판 사용 (LDS-02)
    - Raspberry Pi 4B 4GB
    - 16GB microSD
    - Noetic(Ubuntu 20.04)

## 설정

환경 설정은 터틀봇3 e-Manual을 보며 진행하였다.

[ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

### PC

참고 링크: [https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

1. 위 링크의 1.1.1 ~ 1.1.4를 진행한다.
2. .bashrc 파일에 다음 내용을 추가한다.
    
    ```bash
    export ROS_MASTER_URI=http://{PC_IP}:11311
    export ROS_HOSTNAME={PC_IP}
    export TB3_MODEL=burger
    export TURTLEBOT3_MODEL=burger
    ```
    

### 터틀봇

참고 링크: [https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)

1. 사용하는 보드에 맞는 이미지를 다운로드 한 후, SD카드에 기록한다. (Raspberry Pi Imager 또는 Balena Etcher 사용)
2. SD 카드를 Ubuntu PC에 연결한 후 다음 작업들을 수행한다.
    1. IP 설정 (/etc/netplan/)
    2. SD 카드 공간 확장 (gparted)
    3. hostname 변경 (/etc/hostname)
3. .bashrc 파일에 다음 내용을 추가한다.
    
    ```bash
    export ROS_MASTER_URI=http://{PC_IP}:11311
    export ROS_HOSTNAME={RASPBERRY_PI_IP}
    export TB3_MODEL=burger
    export TURTLEBOT3_MODEL=burger
    export LDS_MODEL=LDS-01 # or LDS-02
    ```
    
4. (추가) LDS-02 센서를 사용할 경우 다음 명령어를 입력한다.
    
    ```bash
    sudo apt update
    sudo apt install libudev-dev
    cd ~/catkin_ws/src
    git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
    cd ~/catkin_ws/src/turtlebot3 && git pull
    rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
    cd ~/catkin_ws && catkin_make
    ```
    
5. 다음 링크를 참고하여 OpenCR 펌웨어를 업로드한다.
    
    [https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup)
    

## 테스트

- 터틀봇 서버 시작
    
    ```bash
    # PC에서 수행
    roscore
    ```
    
- 터틀봇 bringup
    
    ```bash
    # Turtlebot에서 수행
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
    
- Teleoperation
    
    키보드로 터틀봇을 조작하는 프로그램. 앞의 두 단계를 먼저 수행해야 한다.
    
    ```bash
    # Turtlebot에서 수행
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```