#include <zephyr/kernel.h>

#define HELLO_INTERVAL_MS 1000

int main() {
  for (;;) {
    printk("Hello, world!\r\n");

    k_msleep(HELLO_INTERVAL_MS);
  }

  return 0;
}
