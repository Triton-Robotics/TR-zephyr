#include "zephyr/drivers/i2c.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>


const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

const struct device *eeprom_dev = DEVICE_DT_GET(DT_NODELABEL(eeprom_target)); // matches overlay node label


uint8_t tx[2] = {0x00, 0xAA};   // {offset, data}
// uint8_t tx[] = {0x12};
uint8_t rx;


int main(void)
{
    
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
}


