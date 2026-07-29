# ThingSet-over-Serial-Shell Minimal Example — Workplan

## Goal

A minimum working example exposing [ThingSet](https://thingset.io) over the Zephyr
shell, reachable on its own dedicated USB-CDC serial port — a plain `src/main.cpp`
app (no inverter control logic), building on lessons from three legacy references
kept under `old/`:

- **`old/old2`** — a plain, working example (ThingSet over CAN) using the
  *current* ThingSet macro API (`THINGSET_ADD_GROUP` / `THINGSET_ADD_ITEM_FLOAT`).
  Used here as the structural template for `main.cpp` / `user_data_objects.h`.
- **`old/old3`** — attempted ThingSet-over-shell, but does not compile. Root cause:
  it uses an **obsolete ThingSet macro API** (`TS_ADD_GROUP`, `TS_ANY_R`,
  `TS_NO_CALLBACK`, with a different argument order) that no longer matches the
  ThingSet SDK revision pinned in this repo's `west.yml`. Its devicetree overlay
  idea (a second, dedicated shell UART) is correct and was reused here.
- **`old/old4`** — a much more complex, no-longer-running full app (single-phase
  inverter + custom comm protocol). Its `src/app.conf` was a useful cross-check
  for which Kconfig options matter, and `old/old4/tools/` (a Python ThingSet-shell
  test client + saved ThingSet spec pages) was useful for manually verifying the
  protocol dialog.

## What was added / changed

- **`src/user_data_objects.h`** (new) — ThingSet object registration, using the
  current macro API:
  - A read-only `Measurements` group (`rV1Low_V`, `rV2Low_V`, `rVHigh_V`,
    `rI1Low_A`, `rI2Low_A`, `rIHigh_A`, `rTemp1_degC`, `rTemp2_degC`), same as
    `old2`.
  - A writable `Config` group with one item, `wBlinkPeriod_s`, so the shell demo
    exercises a full GET **and** SET dialog, not just reads.
- **`src/main.cpp`** (rewritten) — plain example: background task blinks the LED
  and reads temperature sensors, critical task reads voltage/current sensors, no
  inverter/control-loop code, no direct CAN API calls. The LED blink period now
  reads from the ThingSet-writable `blink_period_s` variable, so a shell `SET`
  has a visible effect on the board.
- **`src/app.conf`** (new) — enables ThingSet + the shell transport:
  - `CONFIG_OWNTECH_COMMUNICATION_ENABLE_CAN=y` — this is the **only** Kconfig
    switch in this repo that reaches ThingSet at all (in
    `zephyr/modules/owntech_communication/zephyr/Kconfig` it `select`s
    `THINGSET`, `THINGSET_SDK`, `THINGSET_CAN`, `CAN`, `ISOTP`, `ISOTP_FAST`,
    `ENTROPY_GENERATOR` together — there's no lighter serial-only switch, so CAN
    plumbing is compiled in even though this example doesn't use it).
  - `CONFIG_THINGSET_SHELL=y` — the actual feature requested, plus the
    `CONFIG_SHELL_*` options it needs (`SHELL_CMDS_SELECT` in particular, to
    `select thingset` from the shell — see below).
  - `CONFIG_LOG=n` — avoids log noise interfering with the shell, matching
    `old2`/`old4` (not `old3`'s `LOG_BACKEND_UART=y`, which looked like a
    leftover inconsistency).
- **`src/app.overlay`** (new) — adds a second USB-CDC-ACM UART instance and
  routes `zephyr,shell-uart` to it. The board's own `spin.dts` already sets
  `zephyr,console` and `thingset,can` by default, so the overlay only needs to
  add what's missing, not redeclare those.

No changes were needed to `zephyr/CMakeLists.txt` (already picks up
`src/app.conf`/`src/app.overlay` automatically) beyond the shield-version bump
that was already staged on this branch.

## Build & flash

Repo-local, module-fetching-aware PlatformIO/West build. The system-wide `pio`
(apt package) is broken on this machine (click API mismatch) — use the
PlatformIO-managed one instead:

```sh
~/.platformio/penv/bin/pio run -e USB              # build
~/.platformio/penv/bin/pio run -e USB -t upload     # flash the connected board
```

Both were run against this example: build succeeded (Flash 78.4%, RAM 40.8%
used), and flashing to the connected board succeeded.

## Verifying the ThingSet serial dialog

After flashing, the board enumerates **two** USB-CDC serial ports (same USB
serial number, two interfaces) — e.g. `/dev/ttyACM0` (console) and
`/dev/ttyACM1` (ThingSet shell). Open the second one at 115200 baud.

The Zephyr shell doesn't run ThingSet as the default command context, so you
first select it:

```
select thingset
```

Then standard ThingSet [Text Mode](https://thingset.io/spec/latest/text-mode/)
requests work:

```
?                                    # dump root
?Measurements null                   # list measurement item names
?Measurements/rV1Low_V                # GET a single reading -> :85 <value>
?Config/wBlinkPeriod_s
```

**Gotcha:** the Zephyr shell's own argument tokenizer strips *unescaped* double
quotes before the command reaches the ThingSet parser, so a naive UPDATE like

```
=Config {"wBlinkPeriod_s":0.2}
```

returns `:A0` (Bad Request) — the quotes around the key get eaten by the shell,
leaving invalid JSON. Escape them instead:

```
=Config {\"wBlinkPeriod_s\":0.2}
```

which correctly returns `:84` (Changed), and a follow-up
`?Config/wBlinkPeriod_s` confirms the new value — this was verified on
hardware, along with the LED blink rate now reading from the updated value.

This was confirmed on the connected board using both the interactive shell and
a short pyserial script; `old/old4/tools/thingset_autotest.py` (generic to the
Text Mode protocol, not tied to `old4`'s specific data model) can also be
pointed at the shell port for an automated read-only discovery smoke test.

## `src/tools/thingset_tools.py`

A host-side helper (needs `pyserial`) that wraps the same protocol dialog
behind a small `ThingSetTools` class, so poking at the device doesn't require
remembering the `select thingset` step or the quote-escaping quirk above:

```python
from thingset_tools import ThingSetTools

ts = ThingSetTools("/dev/ttyACM1")
ts.discover()                              # walks the tree, writes thingset_objects.json
ts.read("Measurements/rV1Low_V")
ts.write("Config", {"wBlinkPeriod_s": 0.2})

ts.objects.Measurements.rV1Low_V           # same read, as an attribute
ts.objects.Config.wBlinkPeriod_s = 0.2     # same write, as an attribute
```

`discover()` classifies each item as a group or leaf by checking whether a
plain GET on it returns a JSON object (group) or a scalar/array (leaf) — name
prefixes alone aren't reliable, since reserved groups like `_Reporting` don't
follow the `r`/`w`/`s`/`x`/`Capitalized` convention. `ts.objects` is populated
from that tree and supports tab-completion in IPython/Jupyter. `read_all()`
and `write_values()` operate on the whole discovered tree at once, the latter
rejecting (client-side, before hitting the wire) any target that isn't
classified writable. All of this was exercised against the connected board,
including the nested `_Reporting/mLive/sEnable` case and rejection of a
read-only write attempt.
