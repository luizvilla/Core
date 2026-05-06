# Open-Loop Sine PWM

This standalone example drives the two TWIST legs with complementary sine PWM. It does not use `singlePhaseInverter`; it is the simplest reference for checking PWM generation, duty clamping and serial control.

## Build

```bash
/home/luiz-villa/.platformio/penv/bin/pio run -e USB_open_loop
```

## Serial Controls

- `h`: print help.
- `p`: start power mode.
- `i`: return to idle and stop PWM.
- `u/j`: increase/decrease sine amplitude by 1 V.
- `d/c`: increase/decrease sine amplitude by 5 V.
- `r`: dump scope data.
- `t`: trigger scope capture.

## Test Procedure

1. Build and upload with `pio run -e USB_open_loop -t upload`.
2. Open the serial monitor and press `h`.
3. Press `p` and verify `duty_cycle_1` and `duty_cycle_2` are complementary.
4. Use `u`, `j`, `d`, and `c` to change amplitude and verify the duty clamp stays within `[0.1, 0.9]`.
5. Press `i` and verify PWM stops.
6. Force or simulate overcurrent above 8 A and verify the state enters `ERRORMODE`.
