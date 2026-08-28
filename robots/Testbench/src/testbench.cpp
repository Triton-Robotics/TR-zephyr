#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

static const struct spi_dt_spec spi1_spec = SPI_DT_SPEC_GET(
    DT_NODELABEL(spi1_loopback_dev),
    SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    0);

static const struct device *spi2_dev =
    DEVICE_DT_GET(DT_NODELABEL(spi2));

static struct spi_config spi2_cfg = {
    .frequency = 375000,
    .operation = SPI_WORD_SET(8)
               | SPI_TRANSFER_MSB
               | SPI_OP_MODE_SLAVE,
};

static uint8_t tx_buf[] = {
    0xDE, 0xAD, 0x12, 0x34
};

static uint8_t rx_buf[sizeof(tx_buf)] = {0};

static struct spi_buf tx_spi_buf = {
    .buf = tx_buf,
    .len = sizeof(tx_buf),
};

static struct spi_buf_set tx_set = {
    .buffers = &tx_spi_buf,
    .count = 1,
};

static struct spi_buf rx_spi_buf = {
    .buf = rx_buf,
    .len = sizeof(rx_buf),
};

static struct spi_buf_set rx_set = {
    .buffers = &rx_spi_buf,
    .count = 1,
};


void slave_thread(void)
{
	printk("slave thread started\n");

	uint8_t slave_tx_buf[sizeof(rx_buf)] = {0};

	struct spi_buf slave_tx_spi_buf = {
		.buf = slave_tx_buf,
		.len = sizeof(slave_tx_buf),
	};

	struct spi_buf_set slave_tx_set = {
		.buffers = &slave_tx_spi_buf,
		.count = 1,
	};

	int ret = spi_transceive(
		spi2_dev,
		&spi2_cfg,
		&slave_tx_set,
		&rx_set
	);

	printk("spi_transceive returned %d\n", ret);
	printk("slave got: %02x %02x %02x %02x\n",
	       rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
}



K_THREAD_DEFINE(
    slave_tid,
    1024,
    slave_thread,
    NULL,
    NULL,
    NULL,
    5,
    0,
    0
);


int main(void)
{
    printk("SPI1 ready: %d\n", spi_is_ready_dt(&spi1_spec));
    printk("SPI2 ready: %d\n", device_is_ready(spi2_dev));

    printk("SPI2 cfg frequency: %u\n", spi2_cfg.frequency);
    printk("SPI2 cfg operation: 0x%x\n", spi2_cfg.operation);

    /*
     * Give the slave thread plenty of time to arm SPI2.
     */
    k_msleep(100);

    printk("starting SPI1 transfer\n");

    int ret = spi_write_dt(&spi1_spec, &tx_set);

    printk("SPI1 write returned %d\n", ret);

    return 0;
}
