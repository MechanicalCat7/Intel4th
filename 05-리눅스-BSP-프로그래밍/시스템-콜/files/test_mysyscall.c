#include <linux/kernel.h>

long sys_mysyscall(long val)
{
	printk(KERN_INFO "Mysyscall val=%ld\n", val);
	return val * val;
}
