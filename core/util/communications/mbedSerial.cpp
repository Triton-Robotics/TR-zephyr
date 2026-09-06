// Created to make it easier to port mbedOS to Zephyr. Mainly used with referee stuff
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include "mbedSerial.h"

SerialBase::SerialBase(const struct device *dev, USART_TypeDef *ll_usart) : uart_dev(dev), m_ll_usart(ll_usart) {
    k_mutex_init(&m_lock);

    // Despite being limited to 1, we're still using semaphores here because they have to be called in an ISR
    // And Mutexes can't be called in an interupt 
    k_sem_init(&m_rx_sem, 0, 1); 
    k_sem_init(&m_tx_space_sem, 0, 1);
    ring_buf_init(&m_rx_rb, sizeof(m_rx_buf), m_rx_buf);
    ring_buf_init(&m_tx_rb, sizeof(m_tx_buf), m_tx_buf);

    uart_irq_callback_user_data_set(uart_dev, &SerialBase::irq_trampoline, this);
    // This function basically declares: When the interrupt happens, irq_trampoline got that. Just let it know we're going thru THIS 
    // SerialBase object, and uart_dev is the device we're going with 
    uart_irq_rx_enable(uart_dev);  // RX stays on; TX toggles via enable_output
}

USART_TypeDef *SerialBase::usart_getter() const {
    return m_ll_usart;
}

// equivalent of enable_output from Serialbase in mbedOS
int SerialBase::enable_output(bool enable) {
    k_mutex_lock(&m_lock, K_FOREVER);

    if (m_tx_enabled != enable) {
        if (enable && !m_rx_enabled) {
            init_peripheral();
        }

        unsigned int key = irq_lock();  // critical section, core_util_critical_section_enter equivalent
                                        // Basically makes a save state (key) and makes sure that the 
                                        // following steps are NOT interrupted
        if (enable) {
            uart_irq_tx_enable(uart_dev);
            pm_device_busy_set(uart_dev);       // lock deep sleep
        } else {
            uart_irq_tx_disable(uart_dev);
            pm_device_busy_clear(uart_dev);     // unlock deep sleep
        }
        irq_unlock(key); // Restores our savestate (key) and everything resumes as normal 

        m_tx_enabled = enable;

        if (!enable && !m_rx_enabled) {
            deinit_peripheral();
        }
    }

    k_mutex_unlock(&m_lock);
    return 0;
}

bool SerialBase::readable() {
        return !ring_buf_is_empty(&m_rx_rb);
    }

int SerialBase::enable_input(bool enable) {
    k_mutex_lock(&m_lock, K_FOREVER);

    if (m_rx_enabled != enable) {
        if (enable && !m_tx_enabled) {
            init_peripheral();
        }

        unsigned int key = irq_lock();
        if (enable) {
            uart_irq_rx_enable(uart_dev);
        } else {
            uart_irq_rx_disable(uart_dev);
        }
        irq_unlock(key);

        m_rx_enabled = enable;

        if (!enable && !m_tx_enabled) {
            deinit_peripheral();
        }
    }

    k_mutex_unlock(&m_lock);
    return 0;
}

void SerialBase::write_unbuffered(const uint8_t *buf_ptr, size_t length) {
    // Basically just spitting out whatever's already queued 
    uint8_t byte;
    while (ring_buf_get(&m_tx_rb, &byte, 1) == 1) {
        uart_poll_out(uart_dev, byte);
    }
    for (size_t i = 0; i < length; i++) {
        uart_poll_out(uart_dev, buf_ptr[i]);
    }
}

ssize_t SerialBase::write(const void *buffer, size_t length) {
    size_t data_written = 0;
    const uint8_t *buf_ptr = static_cast<const uint8_t *>(buffer);

    if (length == 0) {
        return 0;
    }

    if (k_is_in_isr()) { // Checking if we're in an interrupt or an otherwise critical section
        write_unbuffered(buf_ptr, length);
        return length;
    } 
    
    k_mutex_lock(&m_lock, K_FOREVER);
    while (data_written < length) {
        while (ring_buf_space_get(&m_tx_rb) == 0) { // If there's no space
            k_mutex_unlock(&m_lock);
            k_sem_take(&m_tx_space_sem, K_FOREVER);   // wait for "some space freed" signal
            k_mutex_lock(&m_lock, K_FOREVER);
        }
        size_t n = ring_buf_put(&m_tx_rb, buf_ptr + data_written, length - data_written);
        data_written += n;
        enable_output(true);    // This also locks the mutex, but that doesn't matter. Zephyr doesn't have like a 
                                        // double lock system here, the unlock at the end undoes everything
    }
    k_mutex_unlock(&m_lock);
    return (ssize_t)data_written; 
}

ssize_t SerialBase::read(void *buffer, size_t length) {
    size_t data_read = 0; 
    uint8_t *ptr = static_cast<uint8_t *>(buffer);

    if (length == 0) {
        return 0;
    }

    k_mutex_lock(&m_lock, K_FOREVER);
    while (ring_buf_is_empty(&m_rx_rb)) {
        k_mutex_unlock(&m_lock);
        k_sem_take(&m_rx_sem, K_FOREVER);
        k_mutex_lock(&m_lock, K_FOREVER);
    }
    data_read = ring_buf_get(&m_rx_rb, ptr, length);
    k_mutex_unlock(&m_lock);
    return (ssize_t)data_read;
}

void SerialBase::init_peripheral() {
    int ret = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_RESUME);
    // ret == -ENOSYS if the driver doesn't support PM actions —
    (void)ret;
}

void SerialBase::deinit_peripheral() {
    int ret = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_SUSPEND);
    (void)ret;
}

void SerialBase::irq_handler() {
        if(!uart_irq_update(uart_dev)) return; // Asking if our uart stuff refreshed/is it up to date?

        if(uart_irq_rx_ready(uart_dev)) {
            uint8_t byte;
            while (uart_fifo_read(uart_dev, &byte, 1) == 1) { // While we're reading
                if (ring_buf_put(&m_rx_rb, &byte, 1) == 1) { // If there's space in the ring buffer
                    k_sem_give(&m_rx_sem); // Let it access
                }
            }
        }

        if(uart_irq_tx_ready(uart_dev)) { // Does the TX FIFO have space for a byte?
            uint8_t byte;
            if (ring_buf_get(&m_tx_rb,&byte,1) == 1) { // If we can grab a byte from the ringbuffer
                uart_fifo_fill(uart_dev, &byte, 1); // Transfer out that byte
                k_sem_give(&m_tx_space_sem); // 
            }
            else {
                enable_output(false);
            }
        }
    }