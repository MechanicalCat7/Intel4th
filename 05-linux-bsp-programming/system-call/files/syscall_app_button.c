#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <asm-generic/unistd.h>


void print_button(long val);

int main(int argc, char *argv[])
{
    char *endptr;
	long val = 0;
	long last_val = 0;
	long led_val = 0;
	
	// argument check
	if (argc != 2) {
		printf("Usage: %s <0x00~0xff>\n", argv[0]);
		return 1;
	}
	// argument to long
	val = strtol(argv[1], &endptr, 16);
	if (val < 0 || val > 0xff) {
		printf("Usage: %s <0x00~0xff>\n", argv[0]);
		return 1;
	}

    printf("mysyscall return value = %#04x\n", val);
	led_val = val;
	print_button(val);

	// button input loop
	printf("\nPress the button\n");
	do {
		usleep(100000);
		
		// get button value
		val = syscall(__NR_mysyscall, led_val);
		if (val < 0) {
			perror("syscall");
			return 1;
		}
		
		// if same button pressed, continue
		if (val == last_val) {
			continue;
		}
		last_val = val;
		// if no button pressed, continue
		if (val == 0) {
			continue;
		}
		led_val = val;
		
		// print button value
		print_button(val);

		// if button 7 pressed, exit
		if ((val & 0x80) > 0) {
			putchar('\n');
			break;
		}
	} while (1);

	printf("Program stopped.\n");

    return 0;
}

void print_button(long val)
{
	int i;

	puts("\n0:1:2:3:4:5:6:7");
	for (i = 0; i < 8; ++i) {
		if ((val & (1 << i)) > 0)
			putchar('O');
		else
			putchar('X');

		if (i < 7)
			putchar(':');
		else
			putchar('\n');
	}
}
