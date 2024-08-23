# 디바이스 파일과 모듈

## 디바이스 드라이버(Device Driver)

### 디바이스 파일

리눅스는 하드웨어를 직접 다루지 않고 파일 형태로 관리한다. 이를 **디바이스 파일**이라 한다.

디바이스 파일은 하드웨어의 정보를 담고 있으며, `/dev` 디렉터리 밑에 저장되어있다.

디바이스 파일에는 세 가지 정보가 저장된다.

```bash
# /dev
crw-r--r--   1 root    root     10, 235  8월 19 09:00 autofs
brw-rw----   1 root    disk      8,   0  8월 19 09:00 sda
```

- 디바이스 타입 정보: 디바이스가 문자(c)인지 블록( b)인지 구별
- 주 번호: 제어하려는 디바이스를 구분하기 위한 번호
- 부 번호: 디바이스 드라이버가 용도에 따라 관리하는 번호

다음 명령어로 디바이스 파일을 생성할 수 있다.

```bash
sudo mknod /dev/devfile c 240 1
# mknod <filename> <type> <major_num> <minor_num>
```

### 디바이스 드라이버 종류

- 문자 디바이스 드라이버
    - 처리 단위가 Character(1Byte)인 드라이버.
    - 가장 기본적인 드라이버 형태로, 스트림 지향적임.
    - 예: Keyboard
- 블록 디바이스 드라이버
    - 블록 단위로 처리하는 드라이버
    - 한 번에 큰 단위로 처리해야 하는 경우, 내부에 버퍼를 이용하여 처리
    - 예: Disk driver
- 네트워크 디바이스 드라이버
    - 디바이스 파일 형태로 존재하지 않음
    - 응용 프로그램에서 직접적으로 처리할 수 없음

## 모듈(Module)

### 모듈

커널을 다시 빌드하거나 컴파일 하지 않고 커널의 기능을 확장할 수 있는 오브젝트 파일.

- 커널이 부팅된 이후에도 필요한 순간에 모듈을 가져와 활성화할 수 있다. 
(~ 동적 라이브러리)
- 리눅스 커널 버전과 모듈의 커널 버전이 같아야 한다.

### 모듈 프로그래밍

Hello world를 출력하는 모듈이다.

[hello.c](files/hello.c)

```c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

// 모듈 탑재 시 호출
static int hello_init(void)
{
    printk("Hello, world \n");
    return 0;
}

// 모듈 제거 시 호출
static void hello_exit(void)
{
    printk("Goodbye, world \n");
}

module_init(hello_init);
module_exit(hello_exit);

// 필수: 모듈 라이선스
MODULE_LICENSE("Dual BSD/GPL");
```

모듈 작성 시에는 다음 헤더가 포함되어야 한다.

```c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
```

`module_init()` 과 `module_exit()` 은 각각 모듈 탑재 시와 제거 시에 호출될 callback 함수를 등록한다.

```c
module_init(hello_init);
module_exit(hello_exit);
```

모듈에는 반드시 라이선스 정보가 있어야 하며, 기타 정보는 선택 사항이다.

```c
MODULE_LICENSE("BSD License")      // 모듈 라이선스
MODULE_AUTHOR("KOJ")               // (선택)모듈 제작자
MODULE_DESCRIPTION("Test module")  // (선택)모듈 설명
```

**Makefile**

[Makefile](files/Makefile)

```makefile
MOD = hello
obj-m = $(MOD).o

# ARM 빌드
CROSS = ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
KDIR = /home/ubuntu/pi-bsp/kernel/linux
# x86 빌드
#KDIR := /lib/modules/$(shell uname -r)/build
PWD = $(shell pwd)

default:
        $(MAKE) -C $(KDIR) M=$(PWD) modules $(CROSS)
        cp $(MOD).ko /srv/nfs
clean:
        rm -rf *.ko
        rm -rf *.mod.*
        rm -rf .*.cmd
        rm -rf *.o
        rm -rf modules.order
        rm -rf Module.symvers
        rm -rf $(MOD).mod
        rm -rf .tmp_versions
```

위 코드 중 일부에 대한 설명이다.

```makefile
# 빌드할 모듈
MOD = hello
obj-m = $(MOD).o

# 크로스 컴파일 옵션
CROSS = ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
# 리눅스 커널의 Makefile 경로
KDIR := /home/ubuntu/pi-bsp/kernel/linux
# 모듈 코드가 위치한 디렉터리
PWD = $(shell pwd)

default:
# KDIR로 이동한 후, 변수 M의 값을 PWD로 지정, KDIR에 위치한 Makefile을 이용하여 modules 컴파일한다.
# 커널 모듈은 리눅스 커널 디렉터리에 위치한 Makefile을 이용하여 빌드한다.
        $(MAKE) -C $(KDIR) M=$(PWD) modules
```

**모듈 탑재**

```bash
# 모듈 탑재
sudo insmod <module.ko>
# 모듈 제거
sudo rmmod <module.ko>
```

커널 메세지는 `dmesg` 명령어로 확인할 수 있다.

### 모듈 매개변수

모듈 탑재 시 인자를 넘기고 싶으면 `module_param` 매크로를 사용한다.

[modparam.c](files/modparam.c)

```c
// module_param 매크로가 정의된 헤더
#include <linux/moduleparam.h>

// 값을 저장할 변수
static int onevalue = 1;
static char *twostring = NULL;

// module_param(name, type, permission)
module_param(onevalue, int, 0);
module_param(twostring, charp, 0);
```

module_param의 type 목록은 다음과 같다

| Type | Description |
| --- | --- |
| short | short |
| ushort | unsigned short |
| int | int |
| uint | unsigned int |
| long | long |
| ulong | unsigned long |
| charp | char * |
| bool | int |
| invbool | int |
| intarray | int * |

다음과 같이 사용한다.

```bash
sudo insmod <module.ko> param1=value param2="string"
```