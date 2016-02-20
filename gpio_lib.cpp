#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include "gpio_lib.h"

void gpio_check_exported(int gpio_nr)
{
	char buf[256];
	DIR* dir;

	sprintf(buf, "/sys/class/gpio/gpio%d", gpio_nr);
	dir = opendir(buf);

	if (dir)
		/* Directory exists. */
		closedir(dir);
	else
		export_gpio(gpio_nr);
};

int export_gpio(int gpio_nr)
{
	int fd;
	char buf[256];

	fd = open("/sys/class/gpio/export", O_WRONLY);

	sprintf(buf, "%d", gpio_nr);

	write(fd, buf, strlen(buf));

	close(fd);
};

int unexport_gpio(int gpio_nr)
{
	int fd;
	char buf[256];

	fd = open("/sys/class/gpio/unexport", O_WRONLY);

	sprintf(buf, "%d", gpio_nr);

	write(fd, buf, strlen(buf));

	close(fd);
}

int set_gpio_output(int gpio_nr, PIN_VALUE value)
{
	int fd;
	char buf[256];

	gpio_check_exported(gpio_nr);
	sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio_nr);

	fd = open(buf, O_WRONLY);

	// Set out direction
	write(fd, "out", 3);

	close(fd);

	set_gpio_value(gpio_nr, value);
}

int set_gpio_input(int gpio_nr)
{
	int fd;
	char buf[256];

	gpio_check_exported(gpio_nr);
	sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio_nr);

	fd = open(buf, O_WRONLY);

	// Set in direction
	write(fd, "in", 2);

	close(fd);
}

int set_gpio_value(int gpio_nr, PIN_VALUE value)
{
	int fd;
	char buf[256];

	gpio_check_exported(gpio_nr);
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio_nr);

	fd = open(buf, O_WRONLY);

	if (value == LOW) {
		// Set GPIO low status
		write(fd, "0", 1);
	} else {
		// Set GPIO high status
		write(fd, "1", 1);
	}

	close(fd);
}

int get_gpio_value(int gpio_nr)
{
	int fd;
	char buf[256];
	char value;

	gpio_check_exported(gpio_nr);
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio_nr);

	fd = open(buf, O_RDONLY);

	read(fd, &value, 1);

	close(fd);

	return (value == '1') ? HIGH : LOW;
}
