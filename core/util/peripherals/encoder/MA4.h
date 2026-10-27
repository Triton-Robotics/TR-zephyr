#ifndef MA4_H
#define MA4_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

/** MA4 PWM input. Capture callbacks publish measurements; reads never advance filters. */
class MA4 {
public:
    MA4(const struct pwm_dt_spec *pwm, bool invert = false);
    int init();

    struct Reading {
        // Most recent callback, including rejected measurements.
        uint32_t callbacks = 0;
        uint32_t accepted = 0;
        uint32_t rejected = 0;
        uint32_t period_cycles = 0;
        uint32_t pulse_cycles = 0;
        int status = 0;
        int64_t age_ms = -1;
        bool valid = false;

        // Most recent accepted measurement; angles are -1 when unavailable.
        float period_seconds = 0.0f;
        float pulse_seconds = 0.0f;
        float duty = 0.0f;
        float raw_angle = -1.0f;
        float filtered_angle = -1.0f;
        float average_angle = -1.0f;
    };

    /** Consistent diagnostic snapshot. Measurements expire after 100 ms. */
    Reading reading() const;
    /** Period and pulse width in seconds; zero when unavailable. */
    float period();
    float pulsewidth();
    /** Duty ratio in [0, 1]; zero when unavailable. */
    float dutycycle();
    /** Exponentially filtered angle in [0, 360), or -1 when unavailable. */
    float getEncoderYawPosition();
    /** Circular mean of the last 20 accepted raw angles, or -1 when unavailable. */
    float encoderMovingAverage();
    uint32_t cbCount() const;
    int lastStatus() const;
    uint32_t periodCycles() const;
    uint32_t pulseCycles() const;

protected:
    static void capture_cb(const struct device *dev, uint32_t channel,
                           uint32_t period_cycles, uint32_t pulse_cycles,
                           int status, void *user_data);
    void on_capture(uint32_t period_cycles, uint32_t pulse_cycles, int status);

private:
    static constexpr int64_t stale_ms = 100;
    static constexpr unsigned window_size = 20;
    const struct pwm_dt_spec *_pwm;
    bool _invert;
    uint64_t _cycles_per_second = 0;
    mutable struct k_spinlock _lock = {};
    Reading _reading;
    int64_t _last_valid_ms = 0;
    bool _have_filter = false;
    float _angles[window_size] = {};
    unsigned _index = 0;
    unsigned _count = 0;
};

#endif
