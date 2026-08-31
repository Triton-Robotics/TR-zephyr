/* Quick test to check if USART 3 (normally the REF) can write properly to USART 1 (normally the controller)

This is done with interrupts, NOT with DMA

Make sure PC10 is connected to PA10, and that you have 

CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y

*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define REF_NODE   DT_NODELABEL(usart3)
#define CTRL_NODE  DT_NODELABEL(usart1)

static const struct device *const uart_ref  = DEVICE_DT_GET(REF_NODE);
static const struct device *const uart_ctrl = DEVICE_DT_GET(CTRL_NODE);

/* ISR fires whenever USART1 (controller) has bytes waiting in its FIFO */
static void ctrl_rx_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t c;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	while (uart_fifo_read(dev, &c, 1) == 1) {
		printk("RX <- 0x%02x ('%c')\n", c,
		       (c >= 32 && c < 127) ? c : '.');
	}
}

int main(void)
{
	if (!device_is_ready(uart_ref)) {
		printk("USART3 (ref) not ready\n");
		return -1;
	}
	if (!device_is_ready(uart_ctrl)) {
		printk("USART1 (ctrl) not ready\n");
		return -1;
	}

	uart_irq_callback_user_data_set(uart_ctrl, ctrl_rx_isr, NULL);
	uart_irq_rx_enable(uart_ctrl);

	const char *msg = "Hello from USART3!\r\n";
	int count = 0;

	while (1) {
		printk("TX -> [%d] %s", count, msg);

		for (const char *p = msg; *p != '\0'; p++) {
			uart_poll_out(uart_ref, *p);
		}

		count++;
		k_sleep(K_MSEC(1000));
	}

	return 0;
}