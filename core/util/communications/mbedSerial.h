// Created to make it easier to port mbedOS to Zephyr. Mainly used with referee stuff
// This was supposed to be silly and easy but turned out to be very real, reference the driver in MbedOS when debugging
// that's in mbedOS/drivers/source/BufferedSerial.cpp. Sometimes baseSerial.cpp can also be helpful 
#pragma once
#include "stm32f446xx.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/device.h>


// TODO: CONFIRM THESE VALUES
#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256

class SerialBase {
public:
    explicit SerialBase(const struct device *dev, USART_TypeDef *ll_usart);

    // equivalent of enable_output from Serialbase
    int enable_output(bool enable);

    // counterpart for RX, same pattern as mbed's enable_input
    int enable_input(bool enable);

    ssize_t read(void *buffer, size_t length); 
    
    ssize_t write(const void *buffer, size_t length);

    bool readable();

    USART_TypeDef *usart_getter() const; // Returns the USART typedef of the object 


private:

    /**
    @brief A helper function that calls a particular SerialBase's irq_handler safely. 
    
    @details Needed because Zephyr's UART Drivers (specifically uart_irq_callback_user_data_set(uart_dev,&irq_trampoline, this)) 
    needs a static function. However, irq_handler itself can't be static, since it needs to act on a particular instance of SerialBase, so
    we need this static helper function to bounce us to the right SerialBase Object 
     */
    static void irq_trampoline(const struct device *dev, void *user_data) {
        static_cast<SerialBase *>(user_data)->irq_handler();
    }

    /**
    @brief Checks if Uart RX and TX are ready and then writes to ring buffer, or transfers data out respectively

    @warning Always call this through irq_trampoline. Otherwise you'll get an invalid input error. Read irq_trampoline's comments for
    the details
     */
    void irq_handler();

    void init_peripheral();

    void write_unbuffered(const uint8_t *buf_ptr, size_t length);

    void deinit_peripheral();

    const struct device *uart_dev;
    USART_TypeDef *m_ll_usart;
    
    struct k_mutex m_lock;
    struct k_sem m_rx_sem;
    struct k_sem m_tx_space_sem;
    
    struct ring_buf m_rx_rb;
    struct ring_buf m_tx_rb;

    uint8_t m_rx_buf[RX_BUF_SIZE];
    uint8_t m_tx_buf[TX_BUF_SIZE];

    bool m_tx_enabled = false;
    bool m_rx_enabled = false;
}; 