#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <util/peripherals/encoder/MA4.h>

const struct pwm_dt_spec encoderSpec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_encoder_ch));
MA4 encoder_(&encoderSpec);

void periodic() {
    // One snapshot keeps callback status, raw counts, and angles consistent.
    const auto sample = encoder_.reading();
    static uint32_t previous_callbacks = 0;
    static int64_t previous_time = 0;
    const int64_t now = k_uptime_get();
    const int64_t elapsed = now - previous_time;
    const uint32_t captures = sample.callbacks - previous_callbacks;
    const unsigned capture_hz = elapsed > 0
        ? static_cast<unsigned>(static_cast<uint64_t>(captures) * 1000 / elapsed) : 0;
    previous_callbacks = sample.callbacks;
    previous_time = now;
    printk("Encoder: raw=%.2f filtered=%.2f avg=%.2f valid=%d age=%lldms "
           "cb=%u rate=%uHz ok=%u rejected=%u status=%d period=%u pulse=%u\n",
           static_cast<double>(sample.raw_angle),
           static_cast<double>(sample.filtered_angle),
           static_cast<double>(sample.average_angle),
           static_cast<int>(sample.valid), static_cast<long long>(sample.age_ms),
           sample.callbacks, capture_hz, sample.accepted, sample.rejected, sample.status,
           sample.period_cycles, sample.pulse_cycles);
}

int main(void) {
    printk("MA4 encoder capture diagnostics (STOP mode disabled)\n");
    uint64_t clock_hz = 0;
    int ret = encoder_.init();
    if (ret) {
        printk("Failed to initialize encoder: %d\n", ret);
        return ret;
    }
    if (pwm_get_cycles_per_sec(encoderSpec.dev, encoderSpec.channel, &clock_hz) == 0) {
        printk("Capture clock=%llu Hz; MA4-H nominal frequency=920 Hz\n",
               static_cast<unsigned long long>(clock_hz));
    }
    while (true) {
        periodic();
        k_sleep(K_MSEC(100));
    }
}
