#include "MA4.h"

#include <cerrno>
#include <cmath>

namespace {
constexpr float duty_min = 128.0f / 4351.0f;
constexpr float duty_max = 4223.0f / 4351.0f;
// Allow one sensor clock of endpoint quantization from the capture timer.
constexpr float duty_tolerance = 1.0f / 4351.0f;
constexpr float radians_per_degree = 0.017453292519943295f;
constexpr float alpha = 0.2f;

float wrap(float angle) {
    const float result = angle - 360.0f * floorf(angle / 360.0f);
    return result >= 360.0f ? 0.0f : result;
}
}

MA4::MA4(const struct pwm_dt_spec *pwm, bool invert)
    : _pwm(pwm), _invert(invert) {}

int MA4::init() {
    if (!_pwm || !pwm_is_ready_dt(_pwm)) {
        printk("MA4: pwm dev not ready\n");
        return -ENODEV;
    }
    int ret = pwm_get_cycles_per_sec(_pwm->dev, _pwm->channel, &_cycles_per_second);
    if (ret || _cycles_per_second == 0) {
        printk("MA4: cannot get capture clock: %d\n", ret);
        return ret ? ret : -EINVAL;
    }
    ret = pwm_configure_capture(_pwm->dev, _pwm->channel,
                               _pwm->flags | PWM_CAPTURE_TYPE_BOTH |
                               PWM_CAPTURE_MODE_CONTINUOUS, capture_cb, this);
    if (ret) {
        printk("MA4: configure_capture failed: %d\n", ret);
        return ret;
    }
    ret = pwm_enable_capture(_pwm->dev, _pwm->channel);
    if (ret) printk("MA4: enable_capture failed: %d\n", ret);
    return ret;
}

void MA4::on_capture(uint32_t period_cycles, uint32_t pulse_cycles, int status) {
    const int64_t now = k_uptime_get();
    const auto key = k_spin_lock(&_lock);
    ++_reading.callbacks;
    _reading.period_cycles = period_cycles;
    _reading.pulse_cycles = pulse_cycles;
    if (status == 0 && (period_cycles == 0 || pulse_cycles > period_cycles)) {
        status = -ERANGE;
    }
    const float duty = period_cycles ? static_cast<float>(pulse_cycles) / period_cycles : 0.0f;
    if (status == 0 && (duty < duty_min - duty_tolerance ||
                        duty > duty_max + duty_tolerance)) {
        status = -ERANGE;
    }
    _reading.status = status;
    if (status != 0 || _cycles_per_second == 0) {
        if (status == 0) _reading.status = -EIO;
        ++_reading.rejected;
        k_spin_unlock(&_lock, key);
        return;
    }

    // Restart filters after a signal outage rather than blending in old positions.
    if (!_have_filter || now - _last_valid_ms > stale_ms) {
        _have_filter = false;
        _index = 0;
        _count = 0;
    }
    _last_valid_ms = now;
    ++_reading.accepted;
    _reading.period_seconds = static_cast<float>(period_cycles) / _cycles_per_second;
    _reading.pulse_seconds = static_cast<float>(pulse_cycles) / _cycles_per_second;
    _reading.duty = duty;
    // Clamp endpoint quantization instead of wrapping an invalid duty into a plausible angle.
    float angle = (duty - duty_min) * (360.0f / (duty_max - duty_min));
    angle = fminf(360.0f, fmaxf(0.0f, angle));
    angle = wrap(_invert ? 360.0f - angle : angle);
    _reading.raw_angle = angle;
    if (!_have_filter) {
        _reading.filtered_angle = angle;
        _have_filter = true;
    } else {
        const float error = wrap(angle - _reading.filtered_angle + 180.0f) - 180.0f;
        _reading.filtered_angle = wrap(_reading.filtered_angle + alpha * error);
    }
    _angles[_index] = angle;
    _index = (_index + 1) % window_size;
    if (_count < window_size) ++_count;
    k_spin_unlock(&_lock, key);
}

MA4::Reading MA4::reading() const {
    float angles[window_size];
    const auto key = k_spin_lock(&_lock);
    Reading result = _reading;
    const unsigned count = _count;
    for (unsigned i = 0; i < count; ++i) angles[i] = _angles[i];
    result.age_ms = _have_filter ? k_uptime_get() - _last_valid_ms : -1;
    result.valid = _have_filter && result.status == 0 && result.age_ms <= stale_ms;
    k_spin_unlock(&_lock, key);

    if (!result.valid) {
        result.raw_angle = result.filtered_angle = result.average_angle = -1.0f;
        result.period_seconds = result.pulse_seconds = result.duty = 0.0f;
        return result;
    }
    // Calculate outside the interrupt lock. Each instance owns its own window.
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    for (unsigned i = 0; i < count; ++i) {
        sum_sin += sinf(angles[i] * radians_per_degree);
        sum_cos += cosf(angles[i] * radians_per_degree);
    }
    if (sum_sin * sum_sin + sum_cos * sum_cos > 1.0e-8f) {
        result.average_angle = wrap(atan2f(sum_sin, sum_cos) / radians_per_degree);
    }
    return result;
}

void MA4::capture_cb(const struct device *, uint32_t, uint32_t period_cycles,
                     uint32_t pulse_cycles, int status, void *user_data) {
    static_cast<MA4 *>(user_data)->on_capture(period_cycles, pulse_cycles, status);
}

float MA4::period() { return reading().period_seconds; }
float MA4::pulsewidth() { return reading().pulse_seconds; }
float MA4::dutycycle() { return reading().duty; }
float MA4::getEncoderYawPosition() { return reading().filtered_angle; }
float MA4::encoderMovingAverage() { return reading().average_angle; }
uint32_t MA4::cbCount() const { return reading().callbacks; }
int MA4::lastStatus() const { return reading().status; }
uint32_t MA4::periodCycles() const { return reading().period_cycles; }
uint32_t MA4::pulseCycles() const { return reading().pulse_cycles; }
