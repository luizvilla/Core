# Protecting a new power board

> **Status: planned.** This page describes the process decided in [ADR 0009](adr/0009-safety-api-hardware-protection.md). The tools it names (`tools/safety_check.py`, the protection manifest, the generated tests) are being built on branch `safety-hw`. Each step names what exists today.

This page is for anyone designing a new power shield for the Spin, or adding the devicetree of an existing one. It covers what to do so that the board's limits are protected without the CPU, and how the board joins the safety tests.

## 1. Before layout: route the signals the hardware can watch

The Spin's MCU (STM32G474) can switch the PWM off with no CPU involvement in two ways:

- **Comparator path**: a sensor on a comparator input trips in 16 ns typical, 31 ns max. Each comparator watches **one direction** only.
- **ADC-watchdog path**: any ADC input of ADC1 or ADC2 can trip, once per ADC conversion (≤ 20 µs at 50 kHz).

Design rules:

| Signal | Rule |
|---|---|
| Every switch current and AC current | Route to **two** comparator inputs (one per sign) where the pins allow it, and keep one ADC input for measurement |
| Gate-driver FAULT / DESAT, or a shield window comparator | Route to an HRTIM fault pin: Spin pin 48 (PA15), 41 (PB10), 16 (PC10) or 9 (PC7). It trips in 9 ns typical |
| Relay, contactor or driver enable | Add a pull resistor that holds its **safe level** while the MCU resets: after a watchdog reset the MCU pins float |
| DC bus voltage | A comparator input if one is free; otherwise ADC1 or ADC2 |

Comparator inputs on the Spin header:

| Spin pin | MCU pin | Comparator | Note |
|---|---|---|---|
| 30 | PA1 | COMP1+ | used by I1_LOW on every current shield |
| 37 | PB1 | COMP1+ (alternate) | conflicts with pin 30 |
| 44 | PA7 | COMP2+ | |
| 52 | PA3 | COMP2+ (alternate) | conflicts with pin 44 |
| 29 | PA0 | COMP3+ | conflicts with pin 25 |
| 25 | PC1 | COMP3+ (alternate) | used by I2_LOW on every current shield |
| 31 | PB0 | COMP4+ | |
| 1 | PB11 | COMP6+ | |
| 4, 5 | PB13, PB14 | COMP5+, COMP7+ | taken by HRTIM outputs |

**Check the draft:** write a draft overlay with the sensor pins and the legs, then run

```bash
tools/safety_check.py my_shield_draft.overlay     # planned
```

It runs the same allocation as the build and prints which limits get a comparator, an ADC watchdog or only software, the conflicts, the free pins, and the safe outputs still missing a reset declaration. **Attach the report to the schematic review.**

## 2. Write the shield devicetree

Copy the closest shield in `zephyr/boards/shields/` and declare, in addition to the sensors and legs:

- **a limit for each protected quantity**, in physical units. It is the board's absolute maximum: user code can only tighten it.
- **`safe-state`** for each leg if it is not `"open"`.
- **the safe outputs**, each with its safe level, plus `reset-state-safe;` once you have checked the pull resistor on the schematic.
- **an explicit exception** for any limit that has no hardware path. The build refuses to compile a required protection with no path, unless the exception says so.

Then build. The build-check covers every shield in `zephyr/boards/shields/` automatically. It writes the **protection manifest**, the boot line and the board's protection table in the docs.

## 3. Run the bare-Spin tests (L0): no new code

Build the generic safety test firmware for the new shield, flash it on a bare Spin on the SPOC bench, and run the L0 runner. It generates every case from the manifest: one per limit, sign and path, plus CPU supervision and the edge cases. The board appears in the control center.

## 4. Write the bench file and run the powered tests (L2)

Write the board's **bench file** (`tests/safety/boards/<shield>_<version>.json`, planned). It holds what the devicetree cannot know:

- where the fault switch goes;
- source voltage and current limits;
- which terminals form each output port;
- scope wiring.

The bench sheet and the criteria are generated from it and from the manifest. Run them with the bench app, starting at low voltage behind a series bulb.

## Checklist

- [ ] Switch currents, AC current and DC bus routed per §1; `safety_check.py` report attached to the schematic review
- [ ] Safe outputs pulled to their safe level on the board
- [ ] Shield devicetree: limits, safe states, safe outputs with `reset-state-safe;`, exceptions stated
- [ ] Build passes for the new shield; protection table reviewed
- [ ] L0 on the SPOC bench passes; the board shows in the control center
- [ ] Bench file written; L2 cases pass from low voltage up
