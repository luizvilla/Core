# Grid-Forming With Local Sine

This standalone example uses `singlePhaseInverter` in `FORMING` mode. The voltage/current input is a firmware-generated teaching sine built with `ot_modulo_2pi()` and `ot_sin()`.

## Build

```bash
/home/luiz-villa/.platformio/penv/bin/pio run -e USB_grid_forming
```

## Serial Controls

- `h`: print help.
- `p`: ramp the bridge to startup and enter power mode.
- `i`: return to idle and stop PWM.
- `u/j`: increase/decrease `Vdq_ref.d` by 1 V.
- `d/c`: increase/decrease `Vdq_ref.d` by 5 V.
- `r`: dump scope data.
- `t`: trigger scope capture.

## Test Procedure

1. Build and upload with `pio run -e USB_grid_forming -t upload`.
2. Open the serial monitor and press `h`.
3. Press `p` and verify the duty ramps to 0.5 before entering `POWERMODE`.
4. Watch `local_vgrid`, `Vd_ref`, `Vd_in`, `Id_in`, `Vd_out`, `duty_cycle_1`, and `duty_cycle_2` in scope data.
5. Use `u`, `j`, `d`, and `c` to tune the voltage reference.
6. Press `i` and verify PWM stops.
7. Force or simulate overcurrent above 8 A and verify the state enters `ERRORMODE`.
