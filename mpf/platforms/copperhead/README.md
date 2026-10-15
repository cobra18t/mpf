# Copperhead MPF platform

This initial integration supports USB discovery, switches, solenoids, and simple
single-switch hardware rules. Every device number must use `board-address`, where
`board` is the 32-bit value returned by Copperhead command `0x00` and `address` is
the flattened Copperhead switch or solenoid address. The board number remains
required when only one controller is connected.

```yaml
hardware:
  platform: copperhead

copperhead:
  ports: COM5
  baud: 115200
  poll_hz: 100
  response_timeout_ms: 500ms
  spi_functions:
    - board: 20
      function: 8c16s
      allocation: next_spi0
    - board: 20
      function: switch_matrix
      allocation: next_spi0

switches:
  s_start:
    number: 20-23

coils:
  c_trough:
    number: 20-8

  c_flipper:
    number: 20-15
    platform_settings:
      iflip: true
      iflip_analog_input: 20-40
      iflip_eos_start_count: 255
      high_speed: true
      recycle: 2
```

Multiple ports may be listed. Each connected controller must have a distinct
board number; startup fails if duplicate board numbers are discovered.
Copperhead firmware `0.0.0.9` or newer is required. MPF reads the OPP-style
four-byte firmware version during discovery and reports it in platform info.
Runtime status is checked once per second; any nonzero status stops MPF.
`mpf hardware scan` is read-only for Copperhead: it reads board number,
firmware, status, wings, and the 48-byte MULTI configuration without clearing
outputs or assigning volatile SPI functions. Normal startup logs resolved SPI
functions and dynamic stepper/DC-motor assignments at normal file-log level.

Flattened switch addresses are GPIO `0-47`, SPI `48-191`, and switch matrix
`192-255`. Flattened solenoid addresses are GPIO/iFlip `0-23` and SPI `24-127`.
SPI board functions may use `explicit`, `next_spi0`, or `next_spi1` allocation.
MPF clears and reapplies configured functions during startup and retains the
resolved A/B positions.
The `iflip` setting controls configuration-byte bit 3. If it is false, the
channel uses the independently configured `high_speed` PWM methodology.
Both settings default to false. `recycle` defaults to zero and accepts a
multiplier from 0 through 7. Auto-clear, can-cancel, and delayed-kick are
selected from the MPF hardware-rule type; delayed rules retain Copperhead's
1ms resolution and support delays from 1 through 255ms.
When iFlip is enabled, MPF's `pulse_power` (or `default_pulse_power`) is scaled
to the full `0-255` kick-current-threshold range. Firmware applies maximum kick
power while regulating current against that threshold. Hold power remains a
`0-255` PWM value. With iFlip disabled, pulse and hold power use the normal
solenoid `0-127` PWM range.

An enabled iFlip channel requires `iflip_analog_input` in `board-GPIO` form.
`iflip_eos_start_count` selects the firmware EOS start count from 0 through 255
and defaults to 255.
GPIO 40-47 must belong to an active IFLIP_ADC wing and may be assigned to only
one channel. An iFlip-capable channel with no assignment is explicitly sent
GPIO 255. An assignment may remain configured while `iflip` is false.

MPF physical-EOS rules use raw switch state. Installing a rule sends the EOS
switch through `0x0A`; removing it clears physical EOS and repulse while
preserving the analog input and current-sense EOS start count. For EOS repulse,
`eos_active_ms_before_repulse` is divided by kick time and rounded up. MPF uses
the larger of that multiplier and the configured `recycle`, clamps it to 7,
and restores the configured recycle value when the rule is removed. Physical
EOS requires a nonzero kick time, and repulse is incompatible with pulse delay.

Lamp-matrix lights use `board-lamp` numbers and subtype `matrix`; lamp numbers
are 0-63 in column-times-eight-plus-row order. NeoPixels use
`board-output-pixel` for ordinary RGB lights. MPF expands each pixel to
`board-output-channel`, so `start_channel` and explicit channel configurations
work naturally. MPF discovers the available outputs through the `0x09` MULTI
configuration, then sizes each output to the highest configured MPF pixel,
rounded up to a 16-pixel boundary. Outputs without MPF lights use the minimum
16-pixel length. Each output still occupies a fixed 768-channel firmware
address slot. Before shortening an output, MPF clears and latches the original
full strip so pixels beyond the new length cannot retain startup glow.
Copperhead performs fades via the `0x40` interpolation command;
adjacent changes with equal durations are coalesced at MPF's light
synchronization point.

Servos use `board-GPIO` numbers. GPIO must be on a MULTI wing in wings 4-9.
MPF positions map to 0-255, including position zero, which disables servo PWM.
Positive `speed_limit` values are normalized travel per second and determine
the interpolation time; a speed limit of zero or less moves immediately.

Motor Board steppers and brushed-DC motors both use `board-stepper-index`
numbering. The index is returned by the firmware when the Motor Board function
is assigned. Stepper `platform_settings` require a same-board `home_switch` and
accept `microstep` values 0-3, `current_limit` values 0-127, and `acceleration`
values 0-7. At most four Motor Board steppers may be assigned. Acceleration
defaults to 7 and is scaled by firmware to `(value + 1) << 10` step pulses/s^2.
Homing uses acceleration setting 0 and
runs up to 400 Hz; MPF observes the switch state to complete its homing wait.
Stepper stops disable the driver while leaving current-limit PWM configured.
Per-move `speed` values are step pulses per second regardless of microstep mode
and are capped at 4000. Omitting speed sends a zero-duration interpolation.
Copperhead sends absolute interpolation targets and polls the firmware's signed
32-bit step position every 10 ms during motion. MPF uses that reported position
for completion and retargeting instead of estimating progress from elapsed time.

On shutdown, MPF disables steppers and servos, stops DC motors, clears every
solenoid command, and sends immediate zero values to all 64 matrix lamps and
all 768 pixels across the three NeoPixel outputs.

A DC motor owns bits 6 and 7 of its Motor Board output byte, and MPF rejects a
coil using either flattened solenoid address. `default_power` supplies the
normal motor power; an event's explicit `power` overrides it. Copperhead writes
that power as both kick and hold power, then clears the output when the timed
pulse ends or the motor is stopped.

## Future backlog

- Add runtime stepper microstep selection. For now, `microstep` is applied from
  `platform_settings` during initialization and remains fixed while MPF runs.
- Add runtime stepper current-limit control. For now, `current_limit` is applied
  from `platform_settings` during initialization and remains fixed while MPF
  runs.
