#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/gpio.h>


#define CALL_DEV_NAME	"calldev"
#define CALL_DEV_MAJOR	230

#define GPIO_HIGH 1
#define GPIO_LOW 0
#define GPIO_COUNT 8

#define DEBUG 1


static int gpio_led_init(void);
static void gpio_led_set(int val);
static void gpio_led_free(void);
static int gpio_key_init(void);
static int gpio_key_get(void);
static void gpio_key_free(void);

static int gpio_led[] = {6, 7, 8, 9, 10, 11, 12, 13};
static int gpio_key[] = {16, 17, 18, 19, 20, 21, 22, 23};


/* driver */
static int call_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

#if DEBUG
	int num0 = MAJOR(inode->i_rdev);
	int num1 = MINOR(inode->i_rdev);
	printk("%s: call open -> major : %d\n", THIS_MODULE->name, num0);
	printk("%s: call open -> minor : %d\n", THIS_MODULE->name, num1);
#endif

	// initialize gpio
	ret = gpio_led_init();
	if (ret < 0)
	{
		return ret;
	}
	ret = gpio_key_init();
	if (ret < 0)
	{
		return ret;
	}
	printk("%s: GPIO initialized\n", THIS_MODULE->name);

	return 0;
}

static int call_release(struct inode *inode, struct file *filp)
{
#if DEBUG
	printk("%s: call release\n", THIS_MODULE->name);
#endif
	gpio_led_set(0);
	gpio_led_free();
	gpio_key_free();

	return 0;
}

static ssize_t call_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
#if DEBUG
	printk("%s: call read -> buf : %08X, count : %08X\n", THIS_MODULE->name, (unsigned int)buf, count);
#endif
	char kernel_buf = gpio_key_get();
	int ret;

	ret = copy_to_user(buf, &kernel_buf, sizeof(kernel_buf));
//	ret = put_user(kernel_buf, buf);
	if (ret < 0)
	{
		return ret;
	}
	
	return count;
}

static ssize_t call_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
#if DEBUG
	printk("%s: call write -> buf : %08X, count %08X\n", THIS_MODULE->name, (unsigned int)buf, count);
#endif
	char kernel_buf;
	int ret;

	ret = copy_from_user(&kernel_buf, buf, sizeof(kernel_buf));
//	ret = get_user(kernel_buf, buf);
	if (ret < 0)
	{
		return ret;
	}

	gpio_led_set(kernel_buf);

	return count;
}

static long call_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
#if DEBUG
	printk("%s: call ioctl -> cmd : %08X, arg : %08X\n", THIS_MODULE->name, cmd, (unsigned int)arg);
#endif

	return 0x53;
}

static loff_t call_llseek(struct file *filp, loff_t off, int whence)
{
#if DEBUG
	printk("%s: call llseek -> off : %08X, whence : %08X\n", THIS_MODULE->name, (unsigned int)off, whence);
#endif

	return 0x23;
}

static struct file_operations call_fops =
{
	.owner	= THIS_MODULE,
	.open	= call_open,
	.release	= call_release,
	.read	= call_read,
	.write	= call_write,
	.unlocked_ioctl	= call_ioctl,
	.llseek	= call_llseek,
};

static int call_init(void)
{
	int result;

	printk("%s: call init\n", THIS_MODULE->name);
	result = register_chrdev(CALL_DEV_MAJOR, CALL_DEV_NAME, &call_fops);
	if (result < 0)
	{
		return result;
	}

	return 0;
}

static void call_exit(void)
{
	printk("%s: call exit\n", THIS_MODULE->name);
	unregister_chrdev(CALL_DEV_MAJOR, CALL_DEV_NAME);
}

/* gpio */
static int gpio_led_init(void)
{
    int ret = 0;
    int i;
    char gpio_name[10];

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        sprintf(gpio_name, "gpio%d", gpio_led[i]);
        ret = gpio_request(gpio_led[i], gpio_name);
        if (ret < 0)
		{
            printk(KERN_ERR "%s: Failed Request gpio%d error\n", THIS_MODULE->name, gpio_led[i]);
            return ret;
        }
    }

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        ret = gpio_direction_output(gpio_led[i], GPIO_LOW);
        if (ret < 0)
		{
            printk(KERN_ERR "%s: Failed Direction gpio%d error\n", THIS_MODULE->name, gpio_led[i]);
            return ret;
        }
    }

    return ret;
}

static void gpio_led_set(int val)
{
    int i;

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        gpio_set_value(gpio_led[i], (val >> i) & 1);
    }
}

static void gpio_led_free(void)
{
    int i = 0;

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        gpio_free(gpio_led[i]);
    }
}

static int gpio_key_init(void)
{
    int ret = 0;
    int i;
    char gpio_name[10];

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        sprintf(gpio_name, "gpio%d", gpio_key[i]);
        ret = gpio_request(gpio_key[i], gpio_name);
        if (ret < 0)
		{
            printk(KERN_ERR "%s: Failed Request gpio%d error\n", THIS_MODULE->name, gpio_key[i]);
            return ret;
        }
    }

    for (i = 0; i < GPIO_COUNT; ++i) {
        ret = gpio_direction_input(gpio_key[i]);
        if (ret < 0)
		{
            printk(KERN_ERR "%s: Failed Direction gpio%d error\n", THIS_MODULE->name, gpio_key[i]);
            return ret;
        }
    }

    return ret;
}

static int gpio_key_get(void)
{
    int key_val = 0;
    int i;

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        key_val = key_val + (gpio_get_value(gpio_key[i]) << i);
    }

    return key_val;
}

static void gpio_key_free(void)
{
    int i;

    for (i = 0; i < GPIO_COUNT; ++i)
	{
        gpio_free(gpio_key[i]);
    }
}

/* module info */
module_init(call_init);
module_exit(call_exit);

MODULE_AUTHOR("KCCI-KOJ");
MODULE_DESCRIPTION("Device driver test");
MODULE_LICENSE("Dual BSD/GPL");
