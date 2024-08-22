# 파일 시스템

## Yocto Project

### Yocto란

리눅스 기반 시스템 제작에 필요한 도구들을 제공하는 오픈 소스 프로젝트이다.

임베디드 및 IoT 장치에 들어갈 운영체제를 생성하는데 사용된다.

## 파일 시스템 빌드

### 준비

빌드에 필요한 패키지를 설치한다.

```bash
sudo apt update
sudo apt install gawk wget git diffstat unzip texinfo gcc-multilib build-essential \
chrpath socat libsdl1.2-dev xterm python zstd liblz4-tool
```

Poky 리포지토리를 클론한다.

- poky: Yocto 프로젝트의 레퍼런스 이미지

```bash
git clone -b kirkstone git://git.yoctoproject.org/poky.git
```

### 빌드

라즈베리 파이 4b용 파일 시스템을 빌드한다.

```bash
cd poky
# 라즈베리 파이 레시피 가져오기
git clone -b kirkstone git://git.yoctoproject.org/meta-raspberrypi
# 환경 변수 불러오기
source oe-init-build-env
```

`build/conf/local.conf` 파일에서 다음 부분을 수정한다.

```bash
#MACHINE ??= "qemux84-64"    # 주석 처리
MACHINE ??= "raspberrypi4"   # 새로 추가
```

`build/conf/bblayers.conf` 파일에서 다음 부분을 수정한다.

```
BBLAYERS ?= " \ 
  <poky 절대 경로>/meta \
  ...
  <poky 절대 경로>/meta-raspberrypi \    # 내용 추가
  "
```

빌드할 이미지의 이름을 넣고 빌드한다.

```bash
bitbake <빌드할 이미지명>
```

다음은 기본 제공하는 이미지 중 일부이다:

| Image | Description |
| --- | --- |
| core-image-minimal | 리눅스 구동을 위한 최소한의 패키지를 넣은 이미지 |
| core-image-minimal-dev | 개발을 위한 헤더와 라이브러리를 포함한 이미지 |
| core-image-full-cmdline | 콘솔 환경을 위한 대부분의 기능을 포함한 이미지 |
| core-image-x11 | 기본 X11 환경을 제공하는 이미지 |

### SD 카드에 기록

라즈베리 파이의 SD 카드에 기록하기 전에, 기존 SD 카드 정보를 백업한다.

```bash
# /dev/sdb 내의 모든 데이터를 img 파일로 저장
sudo dd if=/dev/sdb of=raspi_sd_2408140954.img bs=512 count=19931136 status=progress
```

빌드한 파일 시스템을 적용하는 방법 중, 두 가지를 적었다. 둘 중 한 가지 방법을 수행한다.

1. ext3 이미지 파일을 rootfs 파티션에 쓰기
    
    ```bash
    sudo umount /dev/sdb2
    sudo dd if=core-image-minimal-raspberrypi4.ext3 of=/dev/sdb2 bs=1M status=progress
    sync
    ```
    
2. 파티션 포맷 후, bz2 파일을 압축 풀기
    
    ```bash
    sudo umount /dev/sdb2  # rootfs 파티션 마운트 해제
    sudo mkfs.ext3 /dev/sdb2  # 파티션을 ext3 형식으로 포맷
    sudo e2label /dev/sdb2 rootfs  # 파티션 라벨 변경
    sudo mount -o loop -t ext3 /dev/sdb2 /media/ubuntu/rootfs  # 재마운트, 또는 GUI 상에서 수행
    sudo tar jxvf <파일명> -C /media/ubuntu/rootfs  # 시스템 파일 복사
    sync  # 쓰기 버퍼 비우기
    ```
    

## 사용자 정의 레이어

### 레이어(Layer)

**레이어**란, 연관이 있는 **레시피(Recipe)**를 모아둔 것을 말한다.

레이어의 구조는 다음과 같다:

```
meta-*               // 레이어
├─ conf
│  └─ layer.conf     // 환경설정 파일
└─ recipes-*
   └─ component
      └ recipe.bb    // 레시피 파일
```

### 사용자 정의 레이어 추가

1. 기본 레이어와 위 구조를 참고하여 레이어 구조 생성
    
    다음은 예시 구조이다:
    
    ```
    .
    ├── conf
    │   └── layer.conf
    └── recipes-rpilinux
        └── images
            └── rpilinux-image.bb
    ```
    
2. 패키지 등록
    
    `recipes` 내의 레시피 파일을 수정하여 필요한 패키지들을 등록한다.
    
    다음은 `rpilinux-image.bb` 예시 파일이다:
    
    ```
    require recipes-core/images/core-image-minimal-dev.bb 
    
    IMAGE_INSTALL += "libstdc++ mtd-utils" 
    IMAGE_INSTALL += "openssh openssl openssh-sftp-server " 
    IMAGE_INSTALL += "python3" 
    IMAGE_INSTALL += "gcc libgcc glibc"
    ```
    
3. 레이어 경로 등록
    
    `build/conf/bblayers.conf` 파일을 수정한다.
    
    ```
    BBLAYERS ?= " \
        ...
        <poky 절대경로>/meta-rpilinux \    # 내용 추가
        "
    ```
    
4. 빌드
    
    추가한 레시피로 빌드한다.
    
    ```bash
    bitbake rpilinux-image
    ```