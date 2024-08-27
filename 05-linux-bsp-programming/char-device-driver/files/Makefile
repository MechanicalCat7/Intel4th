APP := call_app
MOD := call_dev
OBJ := $(APP).o
obj-m := $(MOD).o

# Raspberry Pi 4B cross compile
CROSS = ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
CC = arm-linux-gnueabihf-gcc
# Raspberry Pi 4B kernel path
KDIR := /home/ubuntu/pi-bsp/kernel/linux
PWD := $(shell pwd)

default:$(OBJ)
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(CROSS)
%.o:%.c
	$(CC) -o $(APP) $<
clean:
	rm -rf *.ko
	rm -rf *.mod.*
	rm -rf .*.cmd
	rm -rf *.o
	rm -rf modules.order
	rm -rf Module.symvers
	rm -rf $(MOD).mod
	rm -rf .tmp_versions
	rm -rf $(APP)
