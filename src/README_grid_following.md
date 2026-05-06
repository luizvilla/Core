# Grid-Following PLL

This standalone example uses `singlePhaseInverter` in `FOLLOWING` mode. The PLL input can be selected at runtime:

- `1`: firmware-generated teaching sine from `ot_modulo_2pi()` and `ot_sin()`.
- `2`: measured grid voltage from `V1_LOW - V2_LOW`.

PWM is enabled only after the PLL reports synchronization and the estimated frequency is within the 50 Hz tolerance.

## Build

```bash
/home/luiz-villa/.platformio/penv/bin/pio run -e USB_grid_following
```

## Serial Controls

- `h`: print help.
- `1`: select local sine as the PLL input.
- `2`: select measured `V1_LOW - V2_LOW` as the PLL input.
- `p`: enter startup and wait for PLL sync before power mode.
- `i`: return to idle and stop PWM.
- `u/j`: increase/decrease `Idq_ref.d` by 0.1 A.
- `d/c`: increase/decrease `Idq_ref.d` by 1 A.
- `r`: dump scope data.
- `t`: trigger scope capture.

## Test Procedure

1. Build and upload with `pio run -e USB_grid_following -t upload`.
2. Open the serial monitor and press `h`.
3. Press `1`, then `p`; verify `sync` becomes `1` before PWM starts.
4. Check `pll_vgrid`, `local_vgrid`, `theta`, `omega`, `sync`, `duty_cycle_1`, and `duty_cycle_2` in scope data.
5. Press `i`, then `2`, then `p`; verify the PLL locks from measured `V1_LOW - V2_LOW`.
6. Use `u`, `j`, `d`, and `c` to tune the `d` current reference.
7. Remove or disturb the PLL input and verify sustained desync returns the state to idle.
8. Force or simulate overcurrent above 8 A and verify the state enters `ERRORMODE`.
