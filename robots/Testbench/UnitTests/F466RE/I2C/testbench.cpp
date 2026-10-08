/*
Remember to set whether you want just one transfer, or if you want a continous stream (singularTransfer == false)

Remember your proper wiring (PB8 to PC12) (PB7 to PB10)

This test should work at both 100khz and 400khz clock speeds

If you're running into issues, remember to enable CONFIG_LOG=y in prj.conf, and all the other relevant log configs, they are very helpful
*/

#define singularTransfer true

#include "zephyr/drivers/i2c.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
const struct device *eeprom_dev = DEVICE_DT_GET(DT_NODELABEL(eeprom_target)); // matches overlay node label


int main(void)
{
    #if singularTransfer
    uint8_t tx[2] = {0x00, 0xAA};   // {offset, data}
    uint8_t rx;
    int ret;    
    if (!device_is_ready(i2c1_dev) || !device_is_ready(eeprom_dev)) {
		printk("device(s) not ready\n");
		return 0;
	}

    ret = i2c_target_driver_register(eeprom_dev);
    printk("target register returned: %d\n", ret);

    i2c_recover_bus(i2c1_dev);
    ret = i2c_write(i2c1_dev, tx, sizeof(tx), 0x50);   // write byte to "eeprom" at 0x50
    printk("write1 ret=%d\n", ret);

    ret = i2c_write(i2c1_dev, tx, 1, 0x50);            // set read pointer back to offset 0
    printk("write2 ret=%d\n", ret);

    ret = i2c_read(i2c1_dev, &rx, 1, 0x50);            // read it back
    printk("Read ret = %d\n", ret);


    printk("i2c1 sends %d \n", tx[1]);
    printk("i2c2 receives %d\n", rx);

    return 0;
    #endif

    #if singularTransfer == false

    if (!device_is_ready(i2c1_dev) || !device_is_ready(eeprom_dev)) {
		printk("device(s) not ready\n");
		return 0;
	}

	int ret = i2c_target_driver_register(eeprom_dev);
	if (ret) {
		printk("target register failed: %d\n", ret);
		return 0;
	}

	ret = i2c_recover_bus(i2c1_dev);
	printk("recover ret=%d\n", ret);

	uint32_t pass = 0, fail = 0;
	uint8_t pattern = 0;

	while (1) {
		uint8_t tx[2] = {0x00, pattern};
		uint8_t rx = 0;

		ret = i2c_write(i2c1_dev, tx, sizeof(tx), 0x50);
		if (ret) {
			printk("[%u] write1 failed: %d\n", fail + pass, ret);
			fail++;
			goto next;
		}

        printk("Tx is %d" , tx[1]);

		ret = i2c_write(i2c1_dev, tx, 1, 0x50);
		if (ret) {
			printk("[%u] write2 (set ptr) failed: %d\n", fail + pass, ret);
			fail++;
			goto next;
		}

		ret = i2c_read(i2c1_dev, &rx, 1, 0x50);
		if (ret) {
			printk("[%u] read failed: %d\n", fail + pass, ret);
			fail++;
			goto next;
		}

		if (rx == pattern) {
            printk("Rx is %d\n", rx);
			pass++;
		} else {
			printk("[%u] MISMATCH: sent 0x%02x, got 0x%02x\n",
			       fail + pass, pattern, rx);
			fail++;
		}

next:
		if ((pass + fail) % 50 == 0) {
			printk("pass=%u fail=%u\n", pass, fail);
		}

		pattern++;
		k_msleep(100);
	}

	return 0;
    #endif
}


