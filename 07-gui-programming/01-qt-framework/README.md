# Qt 프레임워크

## Qt

Qt는 GUI 플랫폼 개발에 사용되는 크로스 플랫폼 프레임워크다. 

Windows, Mac OS, Linux 뿐만 아니라, 임베디드용 OS 및 안드로이드와 같은 모바일 OS도 지원한다.

기본적으로는 C++로 개발되며, Python, QML(Qt Modeling Language), Javascript 등의 언어를 지원한다.

## 설치

### Raspberry Pi

apt를 이용해 Qt 패키지를 설치한다.

```bash
sudo apt update
sudo apt install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
```

아래는 수업 진행을 위한 추가 패키지

```bash
sudo apt-get install qtbase5-dev qtdeclarative5-dev qt5-qmake \
   libqt5gui5 qtscript5-dev qtmultimedia5-dev libqt5multimedia5-plugins \
   qtquickcontrols2-5-dev libqt5network5 cmake build-essential
```

```bash
sudo apt install libfontconfig1-dev libdbus-1-dev libfreetype6-dev \
   libicu-dev libinput-dev libxkbcommon-dev libsqlite3-dev libssl-dev \
   libpng-dev libjpeg-dev libglib2.0-dev libraspberrypi-dev
```

```bash
sudo apt install libqt5charts5 libqt5charts5-dev
```

### Windows & Ubuntu

Windows와 Ubuntu에서는 설치 프로그램을 이용하면 쉽게 설치할 수 있다. 설치 시 Qt 계정을 요구한다.

수업에서는 5.14.2 버전을 설치했다.

[Index of /archive/qt/5.14/5.14.2](https://download.qt.io/archive/qt/5.14/5.14.2/)

최신 버전은 다음 링크에서 다운로드 할 수 있다. (open source 버전)

[Download Qt OSS: Get Qt Online Installer](https://www.qt.io/download-qt-installer-oss)

**수업에 사용한 설치 옵션**

- Windows
    - Qt 5.14.2
        - MinGW
        - Sources
        - Qt *
    - Developer and Designer Tools
- Ubuntu
    - Qt 5.14.2
        - Qt *
    - Developer and Designer Tools

Ubuntu의 경우 설치 후 다음 환경 변수를 추가함

```bash
export PATH=/home/ubuntu/Qt5.14.2/5.14.2/gcc_64/bin:$PATH
```