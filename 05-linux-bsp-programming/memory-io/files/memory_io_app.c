#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>


#define DEVICE_FILENAME	"/dev/calldev"


int main(void)
{
	int dev;
	int ret;
	char buf = 0;
	char last_buf = 0;

	dev = open(DEVICE_FILENAME, O_RDWR|O_NDELAY);
	if (dev < 0)
	{
		perror("open");
		return 1;
	}

	do {
		usleep(100000);

		// read button value
		ret = read(dev, &buf, sizeof(buf));
		
		// check same button pressed
		if (buf == last_buf)
			continue;
		last_buf = buf;
		if (buf == 0)
			continue;

		printf("val = %#04x\n", buf);

		// write led value
		ret = write(dev, &buf, sizeof(buf));
		if (buf == 0x80)
			break;

	} while(1);

	ret = close(dev);

	return 0;
}
