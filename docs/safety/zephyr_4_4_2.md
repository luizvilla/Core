# Safety API on Zephyr 4.4.2

Research for [luizvilla/Core#32](https://github.com/luizvilla/Core/issues/32).
Question: does the Safety API (branch `safety-hw`, ADR 0009) still hold after
[owntech-foundation/Core#160](https://github.com/owntech-foundation/Core/pull/160) bumps Zephyr to 4.4.2?

Desk study only. Nothing was built or flashed. Date: 2026-11-25.

## Verdict

The Safety API holds on 4.4.2, with **one required code change**:

- `safety_trips.c:667` connects the HRTIM fault interrupt with `IRQ_CONNECT(..., IRQ_ZERO_LATENCY)`.
  Since Zephyr 4.3.0 this is a **build error** on Arm. A zero-latency interrupt must use `IRQ_DIRECT_CONNECT()`.
  The fix also builds on 4.0.0, so it can land before the bump.

Everything else the Safety API leans on is unchanged: both wrapped fault entries, the fatal-error hook,
`irq_lock()`/BASEPRI, the zero-latency priority, the linker symbols, the system work queue and the ADC hook.

Two conditions to keep:

- Do not turn on `CONFIG_USE_SWITCH` (new in 4.4, off by default on Cortex-M). With it, Zephyr uses a planned
  `udf` fault inside normal context switches, and our `z_arm_fault` wrapper would stop the PWM on it.
- After the bump, run the safety L0 campaigns on the board again. This study reads source; it does not measure.

**Order:** the Safety API PR can go before the bump, if it carries the `IRQ_DIRECT_CONNECT` change.
The bump PR then needs nothing more for safety. If the bump lands first, the Safety API PR does not build
until that change is in.

## Versions compared

| | Today | After #160 |
|---|---|---|
| PlatformIO platform | `ststm32@19.0.0` (`owntech/pio_extra.ini:15`) | `ststm32@20.0.0` |
| framework-zephyr | `~3.40000.0` (platform.json of ststm32 19.0.0) | Zephyr 4.4.2 |
| Zephyr | **v4.0.0** (`VERSION` of the installed package: 4.0.0) | **v4.4.2** |
| CMSIS core | `cmsis` (CMSIS 5) | `cmsis_6` (rev `06d952b6`) |
| hal_stm32 (OwnTech fork) | `2e589737` (STM32CubeG4 v1.5.1) | `ff0e1eef` (STM32CubeG4 v1.6.1) |

Sources: Zephyr tags `v4.0.0` and `v4.4.2` on github.com/zephyrproject-rtos/zephyr, release notes and
migration guides 4.1 to 4.4 (`doc/releases/`), the PR #160 diff, and the hal_stm32 fork at both revisions.
Line numbers below are at the tag named. "Z" means the Zephyr tree.

## Per item

| Item | Result | 4.0.0 | 4.4.2 |
|---|---|---|---|
| `--wrap=z_arm_fault` | Unchanged (name, signature, caller). New early `return` only with `CONFIG_USE_SWITCH` | Z `arch/arm/core/cortex_m/fault.c:1044`, called from `fault_s.S:102` | Z `fault.c:1025`, called from `fault_s.S:103`; `USE_SWITCH` branch `fault.c:1032-1037` |
| `--wrap=z_fatal_error` | Unchanged | def Z `kernel/fatal.c:85`, called from Z `arch/arm/core/fatal.c:86` | def Z `kernel/fatal.c:85`, called from Z `arch/arm/core/fatal.c:93` |
| weak `k_sys_fatal_error_handler` | Unchanged (`kernel/fatal.c` and `fatal.h` identical) | Z `include/zephyr/fatal.h:68`, weak def Z `kernel/fatal.c:37` | same lines |
| `irq_lock()` / BASEPRI | Unchanged | Z `include/zephyr/arch/arm/asm_inline_gcc.h:44-57` | same lines; new `arch_zli_lock()` (PRIMASK) added after line 106, not used by us |
| Zero-latency priority | Unchanged | Z `include/zephyr/arch/arm/cortex_m/exception.h:47-53` | same file `:50-56` |
| `IRQ_CONNECT` with `IRQ_ZERO_LATENCY` | **Changed: build error** | accepted, Z `include/zephyr/arch/arm/irq.h:116` | `BUILD_ASSERT` Z `irq.h:143-144` |
| `irq_connect_dynamic` with `IRQ_ZERO_LATENCY` | Unchanged (no check) | Z `arch/arm/core/cortex_m/irq_manage.c:243` | same function, `:242-249` |
| `_flash_used` | Unchanged (same formula) | Z `include/zephyr/arch/arm/cortex_m/scripts/linker.ld:515` | same file `:428` |
| `__rom_region_start` / `__rom_region_end` | Unchanged | Z `linker.ld:136`, `:254` | Z `linker.ld:131`, `:270` |
| `_image_ram_start` / `_image_ram_end` | Unchanged | Z `linker.ld:276`, `linker/ram-end.ld:11` | Z `linker.ld:296`, `ram-end.ld:12` |
| System work queue | Unchanged behaviour. New opt-in work timeout | Z `kernel/system_work_q.c:22-37`, `kernel/Kconfig:563-573` | Z `system_work_q.c:22-38`, `kernel/Kconfig:594-631` |
| ADC-watchdog hook | Not Zephyr code. Unaffected | — | — |
| CMSIS registers (`CoreDebug`, `DWT`, `__get_IPSR`...) | Still available (CMSIS 6 keeps `CoreDebug` as deprecated alias) | CMSIS 5 | CMSIS_6 `core_cm4.h:1645`, `:1683` |
| STM32 LL / device header symbols | All still present | Cube G4 1.5.1 | Cube G4 1.6.1 |
| edtlib API used by `protection_manifest.py` | Unchanged | | |

### 1. `--wrap=z_arm_fault` and `--wrap=z_fatal_error`

How the Safety API uses them. `zephyr/modules/owntech_safety_api/zephyr/CMakeLists.txt:15` adds
`-Wl,--wrap=z_arm_fault -Wl,--wrap=z_fatal_error`. `safety_supervision.c:601-622` defines
`__wrap_z_arm_fault()` and `__wrap_z_fatal_error()`. Each calls `safety_emergency_stop()`, stamps the cycle
counter, then calls the real function. This stops the PWM before Zephyr logs the fault (250 to 300 us).

`--wrap` only catches calls between object files. Both calls still cross files:

- `z_arm_fault` is called by `bl z_arm_fault` in `fault_s.S` (Z 4.0.0 `:102`, 4.4.2 `:103`).
  Its signature is the same: `(uint32_t msp, uint32_t psp, uint32_t exc_return, _callee_saved_t *callee_regs)`.
  `_callee_saved_t` still exists (Z `include/zephyr/arch/arm/thread.h:48` at 4.4.2).
  `fault_s.S` differs only by blank lines.
- `z_fatal_error` is defined in Z `kernel/fatal.c:85` and called from `z_arm_fatal_error()` in
  Z `arch/arm/core/fatal.c` (4.0.0 `:86`, 4.4.2 `:93`). `kernel/fatal.c` is identical at both tags.

Order of the fault path is the same: `z_arm_fault` -> `z_arm_fatal_error` (logs the ESF) ->
`z_fatal_error` (logs the reason) -> `k_sys_fatal_error_handler`. In 4.4.2 the ESF dump uses
`EXCEPTION_DUMP()` instead of `LOG_ERR()` (Z `arch/arm/core/fatal.c:26-67`), and can unwind the stack
with `CONFIG_EXCEPTION_STACK_TRACE` (`:89-91`). Both run after our wrapper, so the PWM still stops first.

**New in 4.4.2, `CONFIG_USE_SWITCH`.** Release notes 4.4 (`release-notes-4.4.rst:2181-2183`) present a
new Cortex-M context switch behind `CONFIG_USE_SWITCH`. PR #160 mentions it as a reason for the bump.
With it, `z_arm_fault()` first calls `arm_m_iciit_check()` and returns at once when the fault is Zephyr's own
`udf #0` stub (Z `fault.c:1032-1037`, `arm-m-switch.c:224-257`). That stub runs in normal operation: it
restores an interrupted IT/ICI instruction after a context switch. Our wrapper runs *before* that check,
so it would stop every PWM output on a normal context switch. `USE_SWITCH` has no default
(Z `kernel/Kconfig.smp:14-16`); Cortex-M only selects `USE_SWITCH_SUPPORTED`
(Z `arch/arm/core/Kconfig:52`, `cortex_m/Kconfig:230`). So it is off unless someone turns it on.
If OwnTech ever wants it, the wrapper must first skip a fault whose stacked PC is `arm_m_iciit_stub`.
A simple guard today: `BUILD_ASSERT(!IS_ENABLED(CONFIG_USE_SWITCH))` in `safety_supervision.c`.

### 2. Weak `k_sys_fatal_error_handler`

Declaration Z `include/zephyr/fatal.h:68`:
`void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf);` at both tags.
`fatal.h` is identical between the tags. The weak definition is Z `kernel/fatal.c:37` at both.
Our override is `safety_supervision.c:632`. It reads `esf->basic.pc` and `esf->basic.lr`; the layout of
`struct arch_esf` / `struct __basic_sf` is unchanged (Z `include/zephyr/arch/arm/cortex_m/exception.h:112-122`
at 4.4.2). It also uses `k_current_get()`, `k_is_in_isr()`, `thread->stack_info` and `arch_irq_lock()`:
all unchanged public or arch-interface names.

### 3. Zero-latency interrupts and `irq_lock()` / BASEPRI

`irq_lock()` on Armv7-M is still `__set_BASEPRI_MAX(_EXC_IRQ_DEFAULT_PRIO)`
(Z `asm_inline_gcc.h:44-57`, same lines at both tags). The only change in that file is a new
`arch_zli_lock()` / `arch_zli_unlock()` pair that uses PRIMASK (4.4.2, after line 106). We do not use it.

Priorities are the same: `_EXC_ZERO_LATENCY_IRQS_PRIO 0`, `_EXC_SVC_PRIO` = `CONFIG_ZERO_LATENCY_LEVELS`,
`_EXC_IRQ_DEFAULT_PRIO = Z_EXC_PRIO(_IRQ_PRIO_OFFSET)` (Z `cortex_m/exception.h` 4.0.0 `:41-53`,
4.4.2 `:44-56`). So a thread under `irq_lock()` still does not mask zero-latency interrupts.
The kernel liveness design (`docs/safety/kernel_liveness.md`, `safety_supervision.c:217-238`) still holds.

**Changed: `IRQ_CONNECT` with `IRQ_ZERO_LATENCY` no longer builds.** Zephyr commit
[`381d820f6848`](https://github.com/zephyrproject-rtos/zephyr/commit/381d820f6848) ("arm: irq: fail build
if IRQ_CONNECT is used with ZLI", 2025-07-23, first in v4.3.0) adds, in Z `include/zephyr/arch/arm/irq.h:143-144`:

```c
BUILD_ASSERT(!(flags_p & IRQ_ZERO_LATENCY), \
		"ZLI interrupts must be registered using IRQ_DIRECT_CONNECT()");
```

The reason is in commit [`3b8e1fa8df9a`](https://github.com/zephyrproject-rtos/zephyr/commit/3b8e1fa8df9a)
(2025-06-04): a zero-latency interrupt that goes through the regular ISR wrapper can race with the
preamble and postamble of normal interrupts. The migration guides do not mention it.

On `safety-hw`, `safety_trips.c:667` does exactly this:

```c
IRQ_CONNECT(HRTIM1_FLT_IRQn, 0, hrtim_fault_isr, NULL, IRQ_ZERO_LATENCY);
```

Required change: declare `hrtim_fault_isr` with `ISR_DIRECT_DECLARE()`, return 0 (a zero-latency
ISR must not ask for a reschedule, Z `include/zephyr/irq.h:198`), and connect it with
`IRQ_DIRECT_CONNECT(HRTIM1_FLT_IRQn, 0, hrtim_fault_isr, IRQ_ZERO_LATENCY)`. The ISR only touches
registers (`safety_trips.c:540-556`), so it fits the direct-ISR rules. Both macros exist at 4.0.0, so the
change works before and after the bump.

Other interrupt connections, for the record:

- `owntech_communication/.../Rs485.cpp:314` already uses `IRQ_DIRECT_CONNECT(..., IRQ_ZERO_LATENCY)`. Fine.
- `owntech_hrtim_driver/.../hrtim.c:1516` uses `IRQ_CONNECT` with flags 0. Fine.
- The control task (TIM6/TIM7) is connected with `irq_connect_dynamic(..., IRQ_ZERO_LATENCY)`
  (`owntech_timer_driver/.../stm32_timer_driver.c:109-125`). The dynamic path has no check at either tag
  (Z `irq_manage.c` 4.4.2 `:242-249`), so it still builds. But it is the same pattern upstream now forbids
  for static connections: a zero-latency interrupt through the regular ISR wrapper. This is true on 4.0.0
  as well; it is not new with the bump. Worth a separate look (outside this ticket).

`irq_manage.c` also renames `z_arm_irq_priority_set()` to `arm_irq_priority_set()` and
`arch_irq_enable()`/`disable()`/`is_enabled()` to `arm_irq_*()` (the `arch_` names stay as mappings).
We call none of them directly.

### 4. Linker symbols `_flash_used`, `__rom_region_end`

`safety_selftest.c:55-61` declares `__rom_region_start`, `_flash_used`, `_image_ram_start`, `_image_ram_end`.
The flash CRC reads `_flash_used / 4` words from `__rom_region_start` (`safety_selftest.c:331-332`).
`__rom_region_end` appears only in a comment (`:56`): it is not used.

- `_flash_used = LOADADDR(.last_section) + SIZEOF(.last_section) - __rom_region_start;`
  Z `cortex_m/scripts/linker.ld` 4.0.0 `:515`, 4.4.2 `:428`. Same formula. In 4.4.2 `.last_section`
  uses `KEEP(*(.last_section))` instead of an inline `LONG()`; the size it adds is the same.
- `__rom_region_start = ROM_ADDR;` 4.0.0 `:136`, 4.4.2 `:131`.
- `__rom_region_end` 4.0.0 `:254`, 4.4.2 `:270`.
- `_image_ram_start` 4.0.0 `:276`, 4.4.2 `:296`; `_image_ram_end` in Z `include/zephyr/linker/ram-end.ld`
  (4.0.0 `:11`, 4.4.2 `:12`).
- All are still declared in Z `include/zephyr/linker/linker-defs.h` (4.0.0 `:127-136`, 4.4.2 `:152-161`).

The L0 test `tests/l0/safety_supervision/run.py:311` reads `_flash_used` from `nm`: still valid.

### 5. System work queue (run-time self-tests and kernel heartbeat)

The self-tests use `k_work_schedule()` on a delayable work item every 10 ms (`safety_selftest.c:441-457`),
which also stamps the kernel heartbeat. The repeat-clear window uses `k_work_reschedule()`
(`safety_supervision.c:456`).

- `system_work_q.c`: only change is a `static const` config and a new `.work_timeout_ms` field
  (Z 4.4.2 `:24-28`). Start level is the same (`POST_KERNEL`, `:38`).
- `SYSTEM_WORKQUEUE_PRIORITY` default -1 and stack default 1024 are unchanged
  (Z `kernel/Kconfig` 4.0.0 `:563-573`, 4.4.2 `:603-613`). Core sets the stack to 2304 in `zephyr/prj.conf`.
- `kernel/work.c`: the "is it still queued" test now reads the `K_WORK_QUEUED` flag instead of walking the
  list; the chained-submit test uses `queue->thread_id`. Same behaviour for us.
- New: `CONFIG_WORKQUEUE_WORK_TIMEOUT` (Z `kernel/Kconfig:594-600`). If on, a work item that runs too long
  makes Zephyr **abort the work queue thread**. It has no default (off). Note that
  `SYSTEM_WORKQUEUE_WORK_TIMEOUT_MS` defaults to 5000 when `CONFIG_ASSERT=y` (`:628-631`), and Core sets
  `CONFIG_ASSERT=y`; it still has no effect while `WORKQUEUE_WORK_TIMEOUT` is off. If someone turns it on,
  an aborted system work queue stops the heartbeat, so the kernel-liveness check would latch a watchdog
  fault: safe, but worth knowing.
- Tick rate is unchanged (10 000 Hz, Z `kernel/Kconfig` 4.0.0 `:777`, 4.4.2 `:829`), so the
  `K_USEC(200)` feed timer and the 10 ms period keep their rounding.

### 6. The ADC-watchdog hook in `adc_start()`

The hook is **not** in Zephyr's `drivers/adc/adc_stm32.c`. It is in OwnTech's own ADC driver:
`owntech_adc_driver/zephyr/public_api/adc.c:141` defines a weak `adc_safety_before_start(uint8_t adc_num)`,
and `adc_start()` calls it at `adc.c:256-265`, after the channels, DMA and triggers are set and before the
ADCs start. The Safety API overrides it in `safety_trips.c:444`. A second weak hook,
`spin_data_before_start()` in `owntech_spin_api/.../DataAPI.cpp:93`, is overridden in `safety_data.cpp:24`.

Zephyr's `adc_stm32.c` is not built: `CONFIG_ADC` is not set (checked in the built `.config` of the
`SAFETY_L0_TWIST` env), and `ADC_STM32` lives under the `ADC` menu (Z `drivers/adc/Kconfig.stm32:13-16`).
The OwnTech driver programs the ADCs with STM32 LL calls and registers only. So changes to `adc_stm32.c`
(1947 -> 2435 lines) do not reach us.

The devicetree side belongs to the bump PR, not to the Safety API:
`st,adc-clock-source` became a string in 4.1 (`migration-guide-4.1.rst:140-141`; PR #160 already changes
`spin.dts`). 4.3 requires `clock-names` on `st,stm32-adc` nodes (`migration-guide-4.3.rst:103-106`) and
4.4 removes `resolutions` (`migration-guide-4.4.rst:224`). The PR author reports the empty main builds.

hal_stm32: every `LL_*` and register macro used by the Safety API, the OwnTech ADC driver and the HRTIM
driver is still defined in the new fork revision (checked symbol by symbol against `stm32g474xx.h` and the
`ll_adc`, `ll_hrtim`, `ll_comp`, `ll_dac`, `ll_pwr`, `ll_rcc` headers). The `stm32g474xx.h` diff is typo
fixes in comments and a removed SAI `GCR` field.

### 7. Other Zephyr internals on the branch

Search over `safety-hw` (`zephyr/modules`, `src`, `tests/firmware`) for `z_*`, `arch_*`, `_kernel`,
`__wrap_`/`__real_`, linker symbols and IRQ macros:

| Use | Where | Result |
|---|---|---|
| `arch_irq_lock()` in the fatal handler | `safety_supervision.c:658` | Unchanged |
| `_callee_saved_t`, `struct arch_esf` | `safety_supervision.c:601-633` | Unchanged |
| CMSIS: `CoreDebug->DEMCR`, `DWT->CYCCNT`, `__get_PRIMASK`, `__set_PRIMASK`, `__get_MSP`, `__get_PSP`, `__get_IPSR` | `safety_supervision.c:648,671`, `safety_selftest.c:101,144,181-182,316` | Still there in CMSIS 6. `CoreDebug` is a deprecated alias of `DCB` (CMSIS_6 `core_cm4.h:1590-1683`, inside `#ifndef CMSIS_DISABLE_DEPRECATED`; Zephyr does not define that). Migration guide 4.2 `:796-803` moves Cortex-M to CMSIS_6 |
| `SYS_INIT(..., APPLICATION, n)` | `safety_supervision.c:726`, `safety_selftest.c:361,457` | Unchanged |
| `k_timer` feed at `K_USEC(200)` | `safety_supervision.c:127-133, 345-353` | Unchanged |
| `irq_lock()` gate in the HRTIM driver | `hrtim.c:870-878, 889-896` | Unchanged (BASEPRI, zero-latency still runs) |
| edtlib: `edt.compat2nodes`, `compat2okay`, `node.props[..].val`, `node.children`, `node._node`, `dt.phandle2node`, raw `.value`, `.labels` | `scripts/protection_manifest.py:30-120` | All still present at 4.4.2 (Z `edtlib.py:1158,1246,2275-2276`, `Property.val :759`; `dtlib.py:104-106,371-372,834`). `node._node` is private API: a future rename would break the build step loudly |
| `EDT_PICKLE` CMake variable | `CMakeLists.txt:29` | Unchanged (Z `cmake/modules/dts.cmake:111`) |

Nothing on the branch touches `_kernel`, the scheduler or the context switch.

## What this study does not cover

- It does not build 4.4.2. A build of a safety env on #160 + `safety-hw` would confirm the one build error
  and that nothing else breaks.
- It does not measure. Fault-to-PWM-off time, kernel heartbeat gaps and interrupt latency should be
  measured again with the L0 safety campaigns after the bump.
- MCUboot's default mode changed to swap-using-offset in 4.3 (`migration-guide-4.3.rst:485-487`). That
  affects slot layout and DFU, not the CRC (which covers `_flash_used` bytes of the running image only).
  Worth a check in the bump PR.
