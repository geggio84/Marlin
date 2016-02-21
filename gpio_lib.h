
typedef enum {
  LOW = 0,
  HIGH = 1
} PIN_VALUE;

enum EDGE_VALUE {
  NONE = 0,
  RISING = 1,
  FALLING = 2,
  BOTH = 3
};

void gpio_check_exported(int gpio_nr);
int get_gpio_value(int gpio_nr);
int set_gpio_value(int gpio_nr, PIN_VALUE value);
int set_gpio_input(int gpio_nr);
int set_gpio_output(int gpio_nr, PIN_VALUE value);
int unexport_gpio(int gpio_nr);
int export_gpio(int gpio_nr);
