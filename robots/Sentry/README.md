# Sentry autonomy firmware

Sentry uses the current Infantry hardware configuration and accepts Jetson commands
at all times. No remote, keyboard input, switch position, or CV-enable toggle is
required to drive, aim, or shoot. This behavior applies to the Sentry application;
Infantry retains its remote control loop.

Hardware configuration:

| Subsystem | CAN bus | Motor IDs |
| --- | --- | --- |
| Chassis | CAN1 | Front left 5, front right 1, rear left 2, rear right 4 |
| Gimbal yaw | CAN1 | 3 |
| Gimbal pitch | CAN2 | 5 |
| Shooter | CAN2 | Left flywheel 2, right flywheel 4, indexer 6 |

The application also uses Infantry's turret tuning, pitch travel (-22 to +25
degrees), UART/DMA configuration, clock settings, and memory settings. When the
referee reports no positive chassis power limit, the chassis uses an 80 W budget.

## Jetson commands

UART5 runs at **115200 baud**: STM32 TX is PC12, RX is PD2. Send binary packets
using the existing protocol. Floats are 32-bit IEEE-754, little-endian. Each
packet consists of a one-byte header, the payload below, and a one-byte LRC.
The LRC is `(-sum(payload_bytes)) & 0xff`; the header is excluded.

| Header | Payload order | Total packet size |
| --- | --- | --- |
| `0xDD` | `float desired_x_vel`, `float desired_y_vel`, `float desired_angular_vel`, `uint8 localization_calibration` | 15 bytes |
| `0xCC` | `float desired_yaw_rads`, `float desired_pitch_rads`, `uint8 shoot_status` | 11 bytes |

- Chassis velocities are **chassis-relative**, independent of gimbal yaw. X/Y
  are in m/s; angular velocity is in rad/s. The firmware maps these to
  `vX = desired_x_vel`, `vY = -desired_y_vel`, and
  `vOmega = desired_angular_vel`. Each translation component is limited to
  +/-2.92 m/s, and rotation to +/-8 rad/s before the existing wheel limiter.
  The Y sign preserves Sentry's previous Jetson convention. Chassis-relative
  control replaces the previous gimbal/yaw-oriented drive mode; a bridge that
  sends world-relative or gimbal-relative velocities must transform them first.
- Gimbal commands are absolute yaw/pitch targets in radians, not angular rates.
  Yaw is wrapped to +/-180 degrees, and pitch maps to
  `-desired_pitch_rads` before being limited to -22/+25 degrees.
- `shoot_status == 0` turns the shooter off; any nonzero value requests shooting
  using Infantry's burst shooter configuration and existing readiness/heat
  checks. The protocol has no separate flywheel-only or jam-recovery command.
- `localization_calibration` remains decoded but has no actuator behavior.
  Outgoing embedded input telemetry always advertises `activate_CV = 1` and
  `calibration = 0`.

Send each command stream continuously, with less than **500 ms** between valid
packets. Before the first packet, and whenever that stream reaches 500 ms old:

- An expired/missing `0xDD` command clears chassis velocity and wheel power.
- An expired/missing `0xCC` command puts the gimbal in sleep (zero motor power)
  and turns the shooter off.

The streams expire independently: fresh aiming packets cannot keep old movement
active, and fresh movement packets cannot keep old aiming/shooting active. Only
complete packets with valid checksums and finite float values refresh their
timers. Fresh valid packets automatically resume the corresponding subsystem.

## Build and verification

From the repository root, with the Zephyr environment set up:

```sh
source .venv/bin/activate
make sentry-build-clean
```

The firmware output is `build/Sentry/zephyr/zephyr.elf`. To install it on the
connected STM32 using the existing J-Link setup:

```sh
make sentry-flash
```

The current Makefiles use `./.venv/bin/python -m west` when the local virtual
environment exists. If an older Makefile reports `west: No such file or directory`
after moving the checkout, copy `makefiles/Makefile_linux` to `Makefile` on Linux.
Moved virtualenv activation scripts and installed command launchers can retain
their original absolute paths. You can bypass those launchers with
`./.venv/bin/python -m west --version`. A moved checkout also needs a pristine
build (`make sentry-build-clean`) to regenerate cached source paths.

Host regression tests compile the production Jetson decoder/RX worker and Sentry
control loop with substituted hardware interfaces. They require Python 3 and
GCC or Clang with AddressSanitizer/UndefinedBehaviorSanitizer:

```sh
python3 tests/sentry_autonomy/run.py
```

They cover payload ordering, fragmentation, invalid checksums, NaN/Infinity,
startup, independent 500 ms timeouts, ignored remote inputs, limits, shooting,
and command recovery. They do not validate physical UART/CAN wiring or the
Jetson bridge. After flashing, verify small X/Y/rotation commands and gimbal
targets, then stop each stream separately and confirm the corresponding
actuators stop. This repository contains no Jetson bridge changes; command
drops in that bridge still need to be addressed there.
