# Kernel liveness: how long may the threads stop?

ADR 0009, issue luizvilla/Core#23 (C10). This note answers one question: **how long may the Spin run with no thread running before the Safety API resets it**, and where that number comes from. It also covers the slower question: **how long may the run-time self-tests stop**.

## The problem

The control task runs in a zero-latency interrupt. `irq_lock()` raises BASEPRI and leaves zero-latency interrupts on, so a thread that spins under `irq_lock()` stops every thread while the control task keeps running and feeding the watchdog. C10 case 13 showed this on the bench: no reset and no fault, for minutes.

While the threads are stopped:

| Still running (interrupts and hardware) | Stopped (threads) |
|---|---|
| Comparator and ADC-watchdog trips (hardware) | Run-time self-tests: RAM, flash CRC, clock (system work queue) |
| Control task, software limit check, threshold read-back, plausibility (control interrupt) | Communication (USB, CAN, ThingSet) |
| Watchdog feed (control interrupt) | The application's background tasks |

So the Safety API's own limits stay protected. What is lost first is diagnostic coverage. Protection is lost too if the application runs protection functions in threads: grid voltage and frequency protection, anti-islanding, synchronisation.

## What the Safety API does now

The run-time self-test work runs on the system work queue every 10 ms. Each run stamps the cycle counter (`safety_supervision_kernel_alive()`). In the control interrupt, the feed checks the stamp's age. When it is older than `CONFIG_OWNTECH_SAFETY_KERNEL_TIMEOUT_US`, the feed:
1. latches a watchdog fault ("the system work queue did not run", detail `SAFETY_WATCHDOG_KERNEL`);
2. stops the outputs through a hardware fault;
3. stops feeding.

The watchdog then resets the board, and the fault stays latched across the reset. Default: **20 ms**.

- *One heartbeat covers both questions.* A thread spinning under `irq_lock()`, a cooperative thread or work item that never yields, and a work item blocked forever all stop the stamp, and they also stop the self-tests.
- *After a self-test failure* the tests stop and the stamp goes on, so the latched failure is not turned into a reset whose reboot would clear it.
- *Development mode* (run-time self-tests off): nothing stamps, and the check is off.
- `safety_supervision_liveness()` gives the longest gap seen since boot, for measuring a target.

**Why not a faster heartbeat.** The first version had a thread of its own at the highest priority, stamping every 1 ms, with a 5 ms default. L0-TOV (2026-11-19) measured its cost: about 3 points of CPU and about 3 µs of task-start jitter at 100 µs. Two verdicts that passed before failed with it, and passed again with the thread removed (experiment build, same code otherwise). The self-test work runs anyway and costs nothing more. Its 10 ms period sets the floor of the timeout: about 20 ms instead of 5 ms.

## Where the timeout comes from

No standard gives a number for this. The number comes from a chain of requirements:

1. **IEC 61508-2** (functional safety): the time to detect a fault plus the time to reach the safe state must be shorter than the **process safety time**, which is how long the hazard takes to develop after the fault. exida cites this as clause 7.4.5.4 of edition 2. The text was not checked here: confirm the clause in the standard.
2. **IEC 60730-1 Annex H** (Class B software, which the self-tests follow): the fault reaction time and the self-test period are **declared by the manufacturer**, with no fixed number. Not checked in the text (only a preview is public).
3. **The process safety time comes from the product.** For a grid-connected converter, the fastest relevant figures are the grid codes' clearing times:

| Source | Fastest trip | Status here |
|---|---|---|
| IEEE 1547-2018, Tables 11–13 | OV2: 1.20 p.u., clearing time 0.16 s, the same in every category and not adjustable. UV2: 0.45 p.u. at 0.16 s by default, adjustable 0.16–2.0 s | From an unofficial copy of the tables: confirm in the standard and in amendment 1547a-2020 |
| VDE-AR-N 4105 | Total break time (protection plus interface switch) ≤ 200 ms (2011 edition, quoted in test reports). The 2018 edition: over-frequency > 51.5 Hz, trip time < 100 ms | Secondary sources only. The current edition is 2026-03 and may differ |
| EN 50549-1:2019+A1:2023, clause 4.9 | Interface protection settings: a Swiss grid operator's guide gives a default of ≤ 100 ms for its protection relay | Not confirmed in the standard |
| IEC 62109-1/-2, IEC 62477-1 | Product safety of PV and power converters | Not researched yet |

**Reading for the Spin.** The tightest figures are about 100 ms (VDE 4105:2018 over-frequency) and 160 ms (IEEE 1547 OV2). Detection plus reaction must fit inside them, with the converter's own interface switch also counted in VDE's 200 ms. A 100 ms kernel timeout would use the whole budget, so it is too long for an application whose grid protection runs in threads. The 20 ms default uses a fifth of a 100 ms budget and an eighth of 160 ms; the reset that follows takes under a millisecond to stop the outputs (they stop at the latch, before the reset). An application that needs less than about 20 ms must run its protection in the control task, not in threads.

## Measured on the Spin (2026-11-19)

Bench: CUT Spin on the SPOC board, Twist 1.4.2 thresholds, PWM armed on units A and C at 200 kHz, control task at 100 µs. Test firmware `tests/firmware/safety_l0`.

| Load | Longest heartbeat gap (10 ms period) |
|---|---|
| Boot and USB enumeration, idle | 10.50 ms |
| Arming (ADC-watchdog self-tests, comparator functional tests) | 10.60 ms |
| 2 minutes of ThingSet traffic over USB (2366 requests) | 10.63 ms |

The 20 ms default is 1.9 times the worst gap measured. (With the 1 ms thread of the first version, the worst gap was 1.49 ms over 3 minutes of traffic and the fault paths. With a 10 ms period, a gap of 11 ms was measured on the self-test counter.) Each application should measure its own target: its threads, its flash writes, its communication. Read `safety_supervision_liveness()` under the heaviest load, and set the timeout between that figure and its process safety time budget. Flash erase on the system work queue (settings, NVS) needs checking: a page erase stalls the CPU for about 20 ms.

## Bench check (C10)

| Case | Result |
|---|---|
| 13: thread spins under `irq_lock()` | reset; watchdog fault "the system work queue did not run for 20 ms" latched |
| 14: work item blocks forever on the system work queue | same |
| 15: work item spins on the system work queue (cooperative) | same |
| 2, 3, 4: undefined instruction, interrupts masked (PRIMASK), control task stuck | unchanged: fatal error 36 / watchdog reset, "the control task stopped" |

## Sources

- [IEEE 1547-2018 tables (unofficial copy)](https://www.scribd.com/document/548877284/IEEE-Std-1547-2018-Copy); [NRECA guide to IEEE 1547-2018](https://www.cooperative.com/programs-services/bts/Documents/Reports/NRECA-Guide-to-IEEE-1547-2018-March-2019.pdf)
- [TU Graz presentation on VDE-AR-N 4105:2018](https://tugraz.at/fileadmin/user_upload/tugrazExternal/f560810f-089d-42d8-ae6d-8e82a8454ca9/files/pr/344_PR_Herrmann.pdf); [FIMER VDE-AR-N 4105 test report](https://www.fimer.com/sites/default/files/VDE-AR-N_4105_F4.pdf); [VDE: VDE-AR-N 4105 editions](https://www.vde.com/vde-ar-n-4105-2018)
- [VSE/AES guide (strom.ch) on EN 50549-1 settings](https://www.strom.ch/en/media/15546/download)
- [exida FAQ on IEC 61508-2 diagnostic test interval](https://support.exida.com/kb/faq.php?id=226); [Analog Devices: The Importance of Time](https://ez.analog.com/ez-blogs/b/engineerzone-spotlight/posts/the-importance-of-time)
- [IEC 60730-1 ed. 6 preview (VDE Verlag)](https://www.vde-verlag.de/iec-normen/preview-pdf/info_iec60730-1{ed6.0}b.pdf); [Intertek on IEC/UL 60730-1 Annex H](https://www.intertek.com/blog/2026/05-22-functional-safety-and-iecul-60730-1-annex-h/)

To confirm before a certification argument: the IEC 61508-2 clause, the IEEE 1547 tables against the official text, the current VDE-AR-N 4105 (2026-03) and EN 50549-1 tables, and IEC 62109-2.
