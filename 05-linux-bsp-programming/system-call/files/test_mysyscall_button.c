#include <linux/kernel.h>
#include <linux/gpio.h>

#define GPIO_LOW	0
#define GPIO_HIGH	1
#define GPIO_COUNT	8

long gpio_led_init(void);
void gpio_led_set(long val);
void gpio_led_free(void);
long gpio_button_init(void);
long gpio_button_get(void);
void gpio_button_free(void);

int gpio_led[] = {6, 7, 8, 9, 10, 11, 12, 13};
int gpio_button[] = {16, 17, 18, 19, 20, 21, 22, 23};

long sys_mysyscall(long val)
{
	long ret = 0;
	long button_val = 0;

	// GPIO Write LED
	ret = gpio_led_init();
	if (ret < 0) {
		return ret;
	}
	gpio_led_set(val);
	gpio_led_free();

	// GPIO Read Button
	ret = gpio_button_init();
	if (ret < 0) {
		return ret;
	}
	button_val = gpio_button_get();
	gpio_button_free();

	return button_val;
}

/* LED GPIO 초기화 */
long gpio_led_init(void)
{
	long ret = 0;
	int i;
	char gpio_name[10];

	for (i = 0; i < GPIO_COUNT; ++i) {
		sprintf(gpio_name, "gpio%d", gpio_led[i]);
		ret = gpio_request(gpio_led[i], gpio_name);
		if (ret < 0) {
			printk("Failed Request gpio%d error\n", 6);
			return ret;
		}
	}
	
	for (i = 0; i < GPIO_COUNT; ++i) {
		ret = gpio_direction_output(gpio_led[i], GPIO_LOW);
		if (ret < 0) {
			printk("Failed Direction gpio%d error\n", gpio_led[i]);
			return ret;
		}
	}

	return ret;
}

/* LED GPIO 값 쓰기 */
void gpio_led_set(long val)
{
	int i;

	for (i = 0; i < GPIO_COUNT; ++i) {
		gpio_set_value(gpio_led[i], (val >> i) & 1);
	}
}

/* LED GPIO 점유 해제 */
void gpio_led_free(void)
{
	int i = 0;

	for (i = 0; i < GPIO_COUNT; ++i) {
		gpio_free(gpio_led[i]);
	}
}

/* 버튼 GPIO 초기화 */
long gpio_button_init(void)
{
	long ret = 0;
	int i;
	char gpio_name[10];

	for (i = 0; i < GPIO_COUNT; ++i) {
		sprintf(gpio_name, "gpio%d", gpio_button[i]);
		ret = gpio_request(gpio_button[i], gpio_name);
		if (ret < 0) {
			printk("Failed Request gpio%d error\n", gpio_button[i]);
			return ret;
		}
	}

	for (i = 0; i < GPIO_COUNT; ++i) {
		ret = gpio_direction_input(gpio_button[i]);
		if (ret < 0) {
			printk("Failed Direction gpio%d error\n", gpio_button[i]);
			return ret;
		}
	}

	return ret;
}

/* 버튼 GPIO 값 읽기 */
long gpio_button_get(void)
{
	long button_val = 0;
	int i;

	for (i = 0; i < GPIO_COUNT; ++i) {
		button_val = button_val + (gpio_get_value(gpio_button[i]) << i);
	}

	return button_val;
}

/* 버튼 GPIO 점유 해제 */
void gpio_button_free(void)
{
	int i;

	for (i = 0; i < GPIO_COUNT; ++i) {
		gpio_free(gpio_button[i]);
	}
}
