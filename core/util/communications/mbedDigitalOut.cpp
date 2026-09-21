#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

class DigitalOut {
public:
    DigitalOut(const struct gpio_dt_spec spec) : spec_(spec) {
        gpio_pin_configure_dt(&spec_, GPIO_OUTPUT);
    }

    void write(int value) { gpio_pin_set_dt(&spec_, value); }

    DigitalOut& operator=(int value) { write(value); return *this; }
    int read() { return gpio_pin_get_dt(&spec_); } // Might be safe to make this a bool

    void toggle() {
        gpio_pin_toggle_dt(&spec_);
    }
private:
    struct gpio_dt_spec spec_;
};