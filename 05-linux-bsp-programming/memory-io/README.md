# 메모리 읽기/쓰기

## 메모리 공간

- 응용 프로그램에서 읽기, 쓰기 작업을 할 때 사용하는 버퍼는 사용자 메모리 공간에 위치함.
- 응용 프로그램에서 디파이스 파일에 **읽기**를 수행할 경우, 커널은 사용자 메모리 공간에 위치한 버퍼에 **쓰기** 작업을 수행함. (Kernel → User)
- 응용 프로그램에서 디바이스 파일에 **쓰기**를 수행할 경우, 커널은 사용자 메모리 공간에 위치한 버퍼에서 **읽기** 작업을 수행함. (User → Kernel)
- 커널이 사용자 메모리 공간의 데이터를 조작해야 하는 경우, 커널 메모리 공간으로 데이터를 복사한 후 처리한 다음, 사용자 메모리 공간에 기록하는 방식으로 수행한다.
(사용자 메모리 공간을 직접 조작하면 안된다!)

### 메모리 I/O 함수

- copy_to_user
    
    ```c
    #include <asm/uaccess.h>
    
    int copy_to_user(void __user *to, const void *from, unsigned long n);
    ```
    
    커널 메모리 공간의 데이터를 사용자 메모리 공간에 복사하는 함수. (Kernel → User)
    
    to와 from은 각각 커널 버퍼의 주소와 사용자 버퍼의 주소이고, n은 복사할 byte 수다.
    
- copy_from_user
    
    ```c
    #include <asm/uaccess.h>
    
    int copy_from_user(void *to, const void __user *from, unsigned long n);
    ```
    
    사용자 메모리 공간의 데이터를 커널 메모리 공간에 복사하는 함수. (User → Kernel)
    
- get_user
    
    ```c
    #include <asm/uaccess.h>
    
    get_user(x, ptr);
    ```
    
    사용자 메모리 공간 ptr의 데이터를 커널 메모리 공간의 변수 x에 복사하는 매크로 함수. (User → Kernel)
    
    x는 커널 변수, ptr은 사용자 메모리 공간의 주소이다.
    
    매크로 함수 호출 시 변수 x의 크기를 확인하여 1·2·4·8 bytes 만큼 데이터를 가져온다.
    
- put_user
    
    ```c
    #include <asm/uaccess.h>
    
    put_user(x, ptr);
    ```
    
    커널 메모리 공간의 데이터 x를 사용자 메모리 공간 ptr에 복사하는 매크로 함수. 
    (Kernel → User)
    

## 예시 프로그램

GPIO 6~13번 핀에 LED 연결, GPIO 16~23번 핀에 버튼이 연결된 라즈베리 파이를 제어하는 프로그램이다.

프로그램을 실행하고 버튼을 누르면 버튼의 값이 16진수 출력이 되고, LED에 불이 들어온다.

마지막 버튼을 누르면 프로그램이 종료된다.

- 디바이스 드라이버
    
    [memory_io_dev.c](files/memory_io_dev.c)
    
- 응용 프로그램
    
    [memory_io_app.c](files/memory_io_app.c)
    

```bash
sudo mknod /dev/calldev c 230 0
sudo chmod 666 /dev/calldev
sudo insmod memory_io_dev.ko
```