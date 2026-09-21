/*
This test is ONLY to make sure your software-side configuration (prj.conf, devicetree/overlay), for the CANBusses are correct.

It just loops back each canbus controller to itself, which is done internally on the board, so no wiring or transceiver needed

*/



#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
 
#define CAN1_ID 0x100
#define CAN2_ID 0x200
 
#define SEND_PERIOD_MS 1000
 
static const struct device *const can1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
static const struct device *const can2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));
 
static void print_frame(const char *who, const struct can_frame *frame)
{
	printk("%s rx: id=0x%03x dlc=%d data=[", who, frame->id, frame->dlc);
	for (int i = 0; i < frame->dlc; i++) {
		printk("%s%02x", i ? " " : "", frame->data[i]);
	}
	printk("]\n");
}

// These could be combined, but for a test I figured it's best to set explicit callbacks rather than a 
// General callback that switches based on device
static void can1_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev); // We include these unused args because all callbacks need to have this template
	ARG_UNUSED(user_data);
	print_frame("CAN1", frame);
}
 
static void can2_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	print_frame("CAN2", frame);
}

// tx callback is identical for both devices, so there's just 1
static void tx_cb(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	if (error != 0) {
		printk("%s send error: %d\n", static_cast<const char *>(user_data), error);
	}
}

static char can1_tag[] = "CAN1";
static char can2_tag[] = "CAN2";
 
int main(void)
{
	int ret;
 
	if (!device_is_ready(can1_dev)) {
		printk("CAN1 device not ready\n");
		return 0;
	}
	if (!device_is_ready(can2_dev)) {
		printk("CAN2 device not ready\n");
		return 0;
	}
 
	// We're setting each canbus to loop its tx to its rx, since this test is for when you don't have a transceiver
	ret = can_set_mode(can1_dev, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printk("Failed to set CAN1 loopback mode (%d)\n", ret);
		return 0;
	}
 
	ret = can_set_mode(can2_dev, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printk("Failed to set CAN2 loopback mode (%d)\n", ret);
		return 0;
	}
 
	// Filters for its own ID
	const struct can_filter filter_self1 = {
		.id = CAN1_ID,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
 
	const struct can_filter filter_self2 = {
		.id = CAN2_ID,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
 
	ret = can_add_rx_filter(can1_dev, can1_rx_cb, NULL, &filter_self1);
	if (ret < 0) {
		printk("Failed to add CAN1 rx filter (%d)\n", ret);
		return 0;
	}
 
	ret = can_add_rx_filter(can2_dev, can2_rx_cb, NULL, &filter_self2);
	if (ret < 0) {
		printk("Failed to add CAN2 rx filter (%d)\n", ret);
		return 0;
	}
 
	/* Controllers start in CAN_STATE_STOPPED - must explicitly start them */
	ret = can_start(can1_dev);
	if (ret != 0) {
		printk("Failed to start CAN1 (%d)\n", ret);
		return 0;
	}
 
	ret = can_start(can2_dev);
	if (ret != 0) {
		printk("Failed to start CAN2 (%d)\n", ret);
		return 0;
	}
 
	printk("CAN1 <-> CAN2 loopback test started\n");
 
	uint8_t counter = 0;
 
	while (1) {
		struct can_frame counter_frame_1 = {
			.id = CAN1_ID,
			.dlc = 1,
			.flags = 0,
			.data = {counter},
		};
		struct can_frame counter_frame_2 = {
			.id = CAN2_ID,
			.dlc = 1,
			.flags = 0,
			.data = {counter},
		};

 
		ret = can_send(can1_dev, &counter_frame_1, K_MSEC(100), tx_cb, can1_tag);
		if (ret != 0) {
			printk("CAN1 send failed (%d)\n", ret);
		}
 
		ret = can_send(can2_dev, &counter_frame_2, K_MSEC(100), tx_cb, can2_tag);
		if (ret != 0) {
			printk("CAN2 send failed (%d)\n", ret);
		}
 
		counter++;
		k_msleep(SEND_PERIOD_MS);
	}
 
	return 0;
}
