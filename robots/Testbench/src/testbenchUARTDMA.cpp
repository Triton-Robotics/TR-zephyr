/* Jank but working DMA UART check, make sure prj.conf has 
CONFIG_DMA=y
CONFIG_UART_ASYNC_API=y
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#define REF_NODE   DT_NODELABEL(usart3)
#define CTRL_NODE  DT_NODELABEL(usart1)

static const struct device *const uart_ref  = DEVICE_DT_GET(REF_NODE);
static const struct device *const uart_ctrl = DEVICE_DT_GET(CTRL_NODE);

/* Double-buffered DMA RX buffers for USART1 */
#define RX_BUF_LEN 64
static uint8_t rx_buf_a[RX_BUF_LEN];
static uint8_t rx_buf_b[RX_BUF_LEN];

/* Decouples the fast DMA callback from the slow printk() consumer */
RING_BUF_DECLARE(rx_rb, 512);

int ret;

static void ctrl_uart_cb(const struct device *dev,
                         struct uart_event *evt,
                         void *user_data)
{
    ARG_UNUSED(user_data);

    switch (evt->type) {

    case UART_RX_RDY:
        printk(">>> UART_RX_RDY: offset=%d len=%d\n",
               evt->data.rx.offset,
               evt->data.rx.len);

        /* Keep your existing ring-buffer code */
        ring_buf_put(&rx_rb,
                     &evt->data.rx.buf[evt->data.rx.offset],
                     evt->data.rx.len);
        break;

    case UART_RX_BUF_REQUEST: {
        printk(">>> UART_RX_BUF_REQUEST\n");

        static bool use_a;
        uint8_t *next = use_a ? rx_buf_a : rx_buf_b;

        use_a = !use_a;

        int ret = uart_rx_buf_rsp(dev, next, RX_BUF_LEN);

        printk(">>> uart_rx_buf_rsp() = %d, buffer=%s\n",
               ret,
               (next == rx_buf_a) ? "A" : "B");

        break;
    }

    case UART_RX_BUF_RELEASED:
        printk(">>> UART_RX_BUF_RELEASED: buf=%s\n",
               (evt->data.rx_buf.buf == rx_buf_a) ? "A" : "B");
        break;

    case UART_RX_DISABLED:
        printk(">>> UART_RX_DISABLED\n");
        break;

    default:
        printk(">>> UART event: %d\n", evt->type);
        break;
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

    printk("USART1 name: %s\n", uart_ctrl->name);
    printk("USART1 ready: %d\n", device_is_ready(uart_ctrl));
    printk("USART1 api: %p\n", (void *)uart_ctrl->api);


    ret = uart_callback_set(uart_ctrl, ctrl_uart_cb, NULL);
    printk("uart_callback_set: %d\n", ret);

    ret = uart_rx_enable(uart_ctrl, rx_buf_a, RX_BUF_LEN,
                     50 * USEC_PER_MSEC);
    printk("uart_rx_enable: %d\n", ret);

	const char *msg = "Hello from USART3!\r\n";
	int count = 0;
	uint8_t drain[64];

	while (1) {
		printk("TX -> [%d] %s", count, msg);

		for (const char *p = msg; *p != '\0'; p++) {
			uart_poll_out(uart_ref, *p);
		}
		count++;

		/* Safe to printk here -- we're in thread context, not the DMA callback */
		uint32_t n;

		while ((n = ring_buf_get(&rx_rb, drain, sizeof(drain))) > 0) {
			for (uint32_t i = 0; i < n; i++) {
				printk("RX <- 0x%02x ('%c')\n", drain[i],
				       (drain[i] >= 32 && drain[i] < 127) ? drain[i] : '.');
			}
		}

		k_sleep(K_MSEC(1000));
	}

	return 0;
}