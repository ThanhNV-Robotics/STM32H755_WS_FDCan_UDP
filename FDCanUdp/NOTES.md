# FDCanUdp — progress notes

Port of `Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino` to the NUCLEO-H755ZI-Q.
Last updated 2026-10-18.

## Target

**Control 2 legs at ≥ 1000 Hz:** one leg per CAN FD network, 6 MD drivers per leg.
- Left leg on FDCAN1 (PD0/PD1): IDs **10, 20, 30, 40, 50, 60**.
- Right leg on FDCAN2 (PB5/PB6): IDs **70, 80, 90, 100, 110, 120**.

The two FDCAN controllers run in parallel in hardware. If both buses are queued at the start of a cycle, the cycle takes as long as one bus (about 850 µs with FULL frames, 650 µs with TAU), not the sum.

## Status

- **Done, works on hardware:** CM4 drives 6 MAB MD drivers (IDs 10–60) over FDCAN1 at 1 Mbps / 8 Mbps BRS. It pings them, enables them in impedance mode with all gains and targets at zero, and runs the full Portenta benchmark (7 steps × 3 s). Then it disables them.
- **Results match the Portenta** within a few µs (see the table below). The bus is the bottleneck, not the MCU.
- **PVT misses solved:** MD replies aren't retransmitted after losing arbitration. Ascending-ID queue order + one in-cycle retry gives 0 failed cycles (details below).
- **Both legs run in parallel at 1 kHz** (2026-10-18): 12 motors, 0 failed cycles with retry. PVT about 800 µs, TAU about 690 µs, FULL about 914 µs (too tight for a retry). Next: interleave L/R queueing, `-O2`, then the soak test.
- **Not started:** the UDP side on CM7 (Portenta `M7.ino`) and the command/state exchange between the cores (Portenta `M4.ino`).

## Hardware

| Item | Setting |
|---|---|
| Board | NUCLEO-H755ZI-Q, stock |
| FDCAN1 pins (left leg) | **PD1 = TX → transceiver TXD**, **PD0 = RX ← transceiver RXD** |
| FDCAN2 pins (right leg) | **PB6 = TX → transceiver TXD**, **PB5 = RX ← transceiver RXD** |
| Transceivers | One external CAN FD transceiver per bus (the Nucleo has none). Standby/silent pin to GND, VCC 5 V, VIO 3.3 V if present. 120 Ω at both ends of each bus. |
| Log output | USART3 = ST-LINK virtual COM port, `/dev/ttyACM0`, 115200 8N1 |
| LEDs | LD1 green PB0, LD3 red PB14 (configured by CM7, driven by CM4) |

## Clock and power — do not change without care

- **Supply must be direct SMPS** (`PWR_DIRECT_SMPS_SUPPLY`, define `USE_PWR_DIRECT_SMPS_SUPPLY` in both `.cproject` files). LDO locked the board once: `ExitRun0Mode()` turned the SMPS off at reset, and the ST-LINK could no longer connect.
  - Recovery: jumper **CN11 pin 5 (VDD) ↔ pin 7 (BOOT0)**, power cycle, STM32CubeProgrammer → *Under reset* → full chip erase, remove the jumper.
- **HSE = 8 MHz bypass** from the ST-LINK MCO (`HSE_VALUE 8000000`). There is no 48 MHz crystal.
- PLL1: 8 / 2 × 200 / 2 → **CM7 400 MHz, CM4 200 MHz**, voltage scale 1. Scale 0 / 480 MHz is not allowed with direct SMPS.
- **FDCAN kernel clock = PLL2Q = 80 MHz** (8 / 2 × 200 / 10), the same as the Portenta.
  - Nominal 1 Mbps: prescaler 1, 1 + 63 + 16 = 80 tq, SJW 16.
  - Data 8 Mbps: prescaler 1, 1 + 7 + 2 = 10 tq, SJW 2.
  - Transmitter delay compensation on (offset = prescaler × DataTimeSeg1), accept-all global filter to RX FIFO0, auto retransmission **on** (needed for PIPE), protocol exception off.
  - Rx FIFO0 16 × 64 B, Tx FIFO 16 × 64 B, 0 standard filters.
  - **FDCAN2 uses the same settings** (CubeMX → FDCAN2): Frame Format **FD with BitRate Switching** (not `FD_NO_BRS`), same bit timing, same FIFOs, auto retransmission on.
- **Message RAM is shared by FDCAN1 and FDCAN2** (2560 words of 32 bit, 10 KB). Each controller needs its own region, set with **Message Ram Offset** (in words):

  | Controller | Offset | Size | Words used |
  |---|---|---|---|
  | FDCAN1 | **0** | 16 Rx + 16 Tx elements × 18 words | 0 – 575 |
  | FDCAN2 | **576** | 16 Rx + 16 Tx elements × 18 words | 576 – 1151 |
  | free | | | 1152 – 2559 |

  - One element with 64 data bytes = 2 header words + 16 data words = 18 words.
  - If FDCAN2's offset is left at 0, it overwrites FDCAN1's FIFOs and both buses break.
  - Recompute the offset whenever FDCAN1's FIFO sizes, element sizes or filter counts change.
- **Dual-core boot sync is on** (`DUAL_CORE_BOOT_SYNC_SEQUENCE` in both `main.c`). CM4 sleeps until CM7 has set up the clocks. CM4's timing depends on this.
- CubeMX "Copy only the necessary library files" (`LibraryCopy=1`): the HAL sources live in `Drivers/`.

## Code map

| File | Core | What |
|---|---|---|
| `CM4/Core/Src/mab_md.c/.h` | CM4 | MD protocol: DWT µs timebase, send/flush/abort, ping (read 0x802), register write 0x40 / read 0x41, impedance-zero config, disable, 0xA0 reply parser, bus diagnostics |
| `CM4/Core/Src/md_bench.c/.h` | CM4 | Benchmark: BENCH[] steps, SEQ/PIPE update, statistics, window printout, LEDs |
| `CM4/Core/Src/fdcan.c` | CM4 | CubeMX init + USER CODE: TDC, global filter, start |
| `CM4/Core/Src/main.c` | CM4 | `MD_Bench_Setup()` once, `MD_Bench_Loop()` in the loop; `Error_Handler` logs its caller address |
| `Common/Src/ipc_log.c`, `Common/Inc/ipc_log.h` | both | Log channel: CM4 `LOG()` → ring buffer in SRAM4 (0x38000000, 8 KB) → CM7 → USART3 |
| `CM7/Core/Src/main.c` | CM7 | Clocks, `IPC_LogInit()` before releasing CM4, forwards the log in the loop |

Protocol rules learned on the Portenta (still apply):
- Firmware 2.4 replies with header `0x00` instead of echoing `0x41`: never check rx[0] on reads, only reject `0xA1` and check the echoed register address.
- Firmware 2.4 silently ignores `0x42`: use `0x40` (WRITE_REGISTER_LEGACY) for all writes. The `0xA0` default response is the acknowledgement.
- Wait ≥ 300 µs between config frames: an MD drops a frame that arrives right after it sent a reply.
- CAN watchdog is 100 ms (fw 2.4) / 200 ms (fw 2.5.4): never pause longer while motors are enabled.
- **An MD does not retransmit a reply that loses arbitration** (found 2026-10-18). If a lower-ID frame starts at the same moment, the reply is lost. So: queue commands in ascending ID order (reversed order loses almost every reply), and resend once to a motor whose reply is missing (see the PVT-miss experiments in the dev log).

## How to build, flash, watch

1. CubeIDE: F5 on both projects after adding files or regenerating, then Project → Clean, build both.
   - If a regenerate adds a peripheral and the build says `undefined reference to HAL_xxx`: CubeIDE's cached file list is stale. Close and reopen the project, or delete the projects from the workspace (*not* the contents on disk) and import them again.
2. Flash: Run configuration on **FDCanUdp_CM7** (Debugger: SWD, access port 0) with **FDCanUdp_CM4.elf added on the Startup tab, Download ticked**. This flashes both cores.
   - Debugging CM4: start the CM7 debug session first and resume it, then attach CM4 (access port 3, Shared ST-LINK, reset behaviour None, Download unticked).
3. Log: `screen -L -Logfile ~/nucleo_bench.txt /dev/ttyACM0 115200`, then press RESET. Quit with Ctrl-A k y. Ctrl-A d only detaches, and the port stays busy (`screen -ls`, then `screen -S <id> -X quit`).

## Benchmark result (2026-10-18, 6 motors, 3 s per step)

| Step | Nucleo avg / p99 / max (µs) | Rate | Portenta avg (µs) / rate |
|---|---|---|---|
| SEQ FULL 1 kHz | 1244 / 1280 / 2400 | 801 Hz | 1226 / 818 Hz |
| PIPE FULL 1 kHz | 851 / 890 / 891 | 1000 Hz | 849 / 1018 Hz |
| PIPE PVT 1 kHz | 749 / 790 / 800 | 1000 Hz | 746 / 1018 Hz |
| PIPE TAU 1 kHz | 650 / 690 / 691 | 1000 Hz | 647 / 1018 Hz |
| PIPE FULL free-run | 846 / 880 / 877 | 1178 Hz | 844 / 1193 Hz |
| PIPE PVT free-run | 752 / 790 / 793 | 1307 Hz | 749 / 1337 Hz |
| PIPE TAU free-run | 650 / 690 / 692 | 1531 Hz | 647 / 1552 Hz |

- The Portenta's rates read about 1.7 % high because of its millis() error. The Nucleo's timebase check reads exactly 1,000,000 µs per 1000 ms.
- TEC / REC / txFail / rxLost = 0 in every step.
- Per-motor PIPE latency (TAU): 158, 220, 365, 432, 576, 642 µs. Replies arrive in CAN ID order.

## Solved issue: PVT misses on motors 40 and 50

**Solved 2026-10-18:** MD replies are not retransmitted after losing arbitration. The fix is ascending-ID queue order plus one in-cycle retry. Details in the dev log ("PVT-miss experiments"). Original analysis below.

- Only the PVT profile (20 B), only motors 40 and 50 (fw 2.4.0 / 2.4.1). FULL and TAU have zero or almost zero misses. The Portenta shows the same.
- Each miss is a 2 ms timeout: the reply never arrives. The likely cause is the "MD drops a frame right after a reply" behaviour, hit by the PVT timing.
- Worst case seen: 15 consecutive missed cycles on motor 50 (≈ 11 ms without an update), free-running.
- Ideas: resend to the missing motors within the same cycle (about 250 µs left at 1 kHz); use FULL or TAU for control; split the motors across FDCAN1 and FDCAN2.

## Is PVT control ready? (assessed 2026-10-18)

**Not yet: stable in the benchmark, not proven for control.**

Shown so far:
- PVT + retry: 0 failed cycles in about 7,000 cycles (3 s at 1 kHz + 3 s free-run, 6 motors).
- Cause of the misses understood (reply not retransmitted after losing arbitration). A miss most likely loses only the reply (feedback); the motor still received the command.

Still open before real use:
1. **Soak test:** 10–30 min at 1 kHz (600k+ cycles), logging worst miss streak, retries per minute, max cycle time. First with zero gains, then with small safe gains and the motors moving (motor-side timing may change under load).
2. **Control API with retry built in** (per leg: commands in → states out, ascending-ID order, miss counter per motor). The retry only exists in `md_bench.c` so far.
3. **Failure policy:** what to do after N cycles without feedback from a motor (hold, disable the leg, report a fault).
4. **PVT has no kp/kd:** the motor keeps the last written gains (currently 0 from the setup). Write kp/kd once (FULL frame or register write) before using PVT, and again on every gain change. If gains change during walking, FULL (850 µs) may be the better choice.
5. **Both legs in parallel**, then soak test that too.

## Next steps

1. ~~Regenerate with FDCAN2~~, ~~FDCAN2 startup code~~, ~~FDCAN2 hardware test~~ (done 2026-10-18).
2. ~~Two legs in parallel~~ (done 2026-10-18: 0 failed cycles at 1 kHz with retry).
3. Cut the cycle time: interleave the L/R queueing (right leg starts about 37 µs late) and build the motor code with `-O2` (about 25 µs CPU overhead per cycle).
4. Soak test both legs: PVT + retry at 1 kHz for 10–30 min (see "Is PVT control ready?").
4. ~~Fix the PVT misses~~: solved with ascending-ID order + in-cycle retry (2026-10-18). Use the retry in the 2-leg control loop.
5. Port the UDP side to CM7 (Portenta `M7.ino`): Ethernet + LwIP on the Nucleo.
6. Command/state exchange between CM7 and CM4 (Portenta `M4.ino`), for example a second shared-memory channel next to the log buffer in SRAM4.
7. Clean up: `CM4/Core/Src/mab_can.c/.h` and the CM7 copies are no longer used.

## Dev log

Newest entries at the bottom. Add one entry per work session: what changed, what was learned, what broke.

### 2026-10-17 — build fixes, FDCAN moved to CM4
- Regenerating with "Add library files as reference" deleted `Drivers/` and linked the HAL into `~/STM32Cube/Repository`. CubeIDE kept building from a stale file list (`undefined reference to HAL_FDCAN_Init`). Fixed by removing and re-importing the projects; switched to "Copy only the necessary library files".
- The regenerate also erased the ping code in `fdcan.c` and zeroed the FDCAN FIFO sizes. Restored them from git and set the FIFO sizes in the `.ioc`.
- Decided: FDCAN runs on **CM4**, CM7 keeps the LEDs and USART3.
- Ported the Portenta ping (read `firmwareVersion` 0x802, accept the `0x00` header from fw 2.4). The old `mab_can` ping expected `0x43` and would have rejected 5 of 6 motors.

### 2026-10-18 — board locked, clock fix, benchmark ported
- The `.ioc` had been set to LDO supply and a 48 MHz HSE crystal. LDO cut the core supply at reset and the ST-LINK could no longer connect. Recovered with BOOT0 (CN11 pin 5 ↔ 7) and a full chip erase. Now: direct SMPS, 8 MHz HSE bypass, CM7 400 MHz / CM4 200 MHz, FDCAN on PLL2Q = 80 MHz.
- Turned on dual-core boot sync so CM4 starts only after CM7 has set the clocks.
- No motor answered at first: an external transceiver with its standby pin pulled low was needed. After that, all 6 motors answered.
- Added the serial log: CM4 → SRAM4 ring buffer → CM7 → USART3 (ST-LINK VCP).
- Ported the full Portenta benchmark (impedance zero + 7 steps). Results match the Portenta within a few µs (table above). The PVT misses on motors 40 and 50 also appear here.

### 2026-10-18 — FDCAN2 for the second leg
- Target set: 2 legs, one CAN FD network each, ≥ 1000 Hz.
- Added FDCAN2 in the `.ioc` on CM4, PB5 (RX) / PB6 (TX), same timing and FIFOs as FDCAN1.
- Two settings had to be fixed in CubeMX: Frame Format was `FD_NO_BRS` (now `FD_BRS`), and Message Ram Offset was 0, overlapping FDCAN1 (now 576).
- Checkpoint commit **`c6d2fde`** made before regenerating. If the regenerate breaks something, restore a file with `git checkout c6d2fde -- <path>`.
- Regenerated. Everything survived: SMPS, HSE bypass, boot sync, USER CODE.
  - CubeMX moved the PLL2 / FDCAN kernel clock setup from `HAL_FDCAN_MspInit` (CM4) to a new `PeriphCommonClock_Config()` in CM7's `main.c`. CM7 runs it before waking CM4. CM4 gets an unused copy.
  - The FDCAN clock enable is now reference-counted in `fdcan.c` (shared by FDCAN1 and FDCAN2).
- Added FDCAN2 startup in `fdcan.c` USER CODE (TDC, accept-all filter, start), as for FDCAN1.
- `mab_md` / `md_bench` no longer use `hfdcan1` directly: `MD_SetBus(&hfdcan2)` selects the controller, and `MD_PrintConfig()` logs which one.
- Hardware test: the same 6 motors (IDs 10–60) moved to FDCAN2, benchmark run on FDCAN2 (`MD_SetBus(&hfdcan2)` in CM4 `main.c`).
  - **Result: FDCAN2 works.** All 6 pinged and enabled; TEC / REC / txFail / rxLost = 0 in every step; timings match FDCAN1 within a few µs.

    | Step | FDCAN2 avg / p99 / max (µs) | Rate |
    |---|---|---|
    | SEQ FULL 1 kHz | 1249 / 1300 / 2389 | 798 Hz |
    | PIPE FULL 1 kHz | 852 / 890 / 891 | 1000 Hz |
    | PIPE PVT 1 kHz | 750 / 790 / 786 | 996 Hz (32 failed) |
    | PIPE TAU 1 kHz | 651 / 690 / 692 | 1000 Hz |
    | PIPE FULL free-run | 849 / 880 / 880 | 1174 Hz |
    | PIPE PVT free-run | 753 / 790 / 786 | 1305 Hz (31 failed) |
    | PIPE TAU free-run | 651 / 690 / 693 | 1529 Hz |

  - The PVT misses on motors 40 and 50 appear on FDCAN2 too (max 4 in a row). Same motors on FDCAN1, FDCAN2 and the Portenta, so it's how those motors behave, not the bus.

### 2026-10-18 — PVT-miss experiments
- Evidence so far: only PVT (20 B) misses, only motors 40 and 50 (queue positions 4 and 5), `invalid 0`, no bus errors. Motor 30 has the same firmware as 40 and never misses. Working theory: an MD drops a frame that arrives right after it sent a reply, and with 20-byte frames the commands for 40/50 land in that window.
- Added 4 benchmark steps (after the original 7, so the run now takes about 35 s):
  - `PIPE PVT pad24 1000 us`: same content zero-padded to 24 bytes. Misses gone or moved → it's frame timing.
  - `PIPE PVT reversed 1000 us`: queue 60 → 10. Misses move to other queue positions → it's position, not motors 40/50.
  - `PIPE PVT retry 1000 us` and `retry free-run`: resend once to a motor whose reply is missing. Replies arrive in queue order, so a reply from a later position means an earlier command was dropped → resend at once. Fallback: no reply for 300 µs → resend to everyone still missing. Logged per motor as "retried N, answered after retry M".
- Result (FDCAN2, log `~/nucleo_fdcan2.txt`):

  | Step | Failed cycles | Motor 40 / 50 missed | Avg / max (µs) |
  |---|---|---|---|
  | PVT 1 kHz (baseline) | 40 | 17 / 23 | 753 / 790 |
  | PVT free-run (baseline) | 116 | 75 / 41 (max 16 in a row) | 756 / 790 |
  | PVT pad24 1 kHz | 2 | 0 / 2 | 783 / 819 |
  | PVT reversed 1 kHz | 1490 (all) | 60/50/40 never answered, 30 almost never | — |
  | PVT retry 1 kHz | **0** | 8 / 7 retried, all answered | 755 / 871 |
  | PVT retry free-run | **0** | 13 / 14 retried, all answered | 758 / 864 |

- **Cause found: the MD drivers do not retransmit a reply that loses arbitration.** Reversed order proves it: while lower-ID commands are still queued, every reply from a higher ID loses and is dropped (60, 50, 40 never answered; the lower the ID, the fewer losses). In normal order, PVT timing makes motor 40/50's reply start at the same moment as a lower-ID frame. Padding to 24 bytes shifts the timing and removes most collisions.
- **Decisions for the control loop:**
  1. Always queue commands in **ascending CAN ID** order.
  2. Use the **in-cycle retry**: 0 failed cycles, with a retried reply about 200 µs late (max 871 µs, still < 1 ms).
  3. Padding is not a fix by itself (2 misses left, depends on exact timing).
- Committed as `4fcd4fc`.

### 2026-10-18 — 2-leg wiring, right leg test
- Assessed PVT readiness: stable in the benchmark, not yet proven for control (see "Is PVT control ready?").
- Wiring now final: **left leg on FDCAN1, IDs 10–60; right leg on FDCAN2, IDs 70–120.**
- Added `MD_SetMotorIds()` (motor IDs were hard-coded to 10–60). `MD_PrintConfig()` now logs the bus and the IDs.
- Test: same benchmark (all 11 steps) on FDCAN2 with the right leg (`MD_SetBus(&hfdcan2)` + `MD_SetMotorIds({70..120})` in CM4 `main.c`).
  - **Result: right leg works, same as the left leg.** TEC / REC / txFail / rxLost = 0 in every step, all config writes and disables acknowledged.

    | Step | Right leg (avg / max µs, failed) | Left leg (avg / max µs, failed) |
    |---|---|---|
    | SEQ FULL 1 kHz | 1252 / 2431, 0 | 1249 / 2389, 0 |
    | PIPE FULL 1 kHz | 860 / 898, 0 | 852 / 891, 0 |
    | PIPE PVT 1 kHz | 758 / 794, 19 | 753 / 790, 40 |
    | PIPE TAU 1 kHz | 658 / 700, 0 | 654 / 696, 0 |
    | PIPE TAU free-run | 657 / 700, 0 (1515 Hz) | 654 / 697, 1 (1521 Hz) |
    | PVT pad24 1 kHz | 787 / 823, 2 | 783 / 819, 2 |
    | PVT reversed | all failed | all failed |
    | PVT retry 1 kHz | 759 / 869, **0** | 755 / 871, **0** |
    | PVT retry free-run | 763 / 869, **0** | 758 / 864, **0** |

  - The PVT misses hit **100 and 110**, queue positions 4 and 5. They are different physical motors from 40/50, so this confirms the cause is queue position / reply timing, not specific motors.
  - Retry: 0 failed cycles; worst reply 869 µs (< 1 ms).
- Committed as `b2db0ba`.

### 2026-10-18 — both legs in parallel
- Refactored `mab_md` for two legs:
  - Each `MD_Motor_t` carries its FDCAN handle.
  - `MD_Leg_t` (name, bus, 6 motors, bus diagnostics) is set up with `MD_LegInit()`, in `mdLegs[2]`.
  - Removed `MD_SetBus`, `MD_SetMotorIds`, `mdMotors[]` and the global diagnostics.
- Rewrote `md_bench` for two legs:
  - **PIPE:** queue all 12 commands (ascending ID per leg) on both controllers, then poll both RX FIFOs in one loop. Retry per leg (reply from a later position → resend the earlier missing one; no reply on a leg for 300 µs → resend all its missing ones).
  - **SEQ:** motor i of both legs at the same time.
  - Per window: full update time for both legs, plus per leg the "done" time (cycle start → that leg's last reply), TEC/REC/txFail/rxLost, and a line per motor (`L id 10`, `R id 70`).
  - Dropped the `pad24` / `reversed` experiments. Added retry steps for FULL, PVT and TAU at 1 kHz plus PVT retry free-run (11 steps, about 33 s).
  - Startup pings both legs before enabling any motor; a leg with missing motors still runs with the ones it has.
- CM4 `main.c`: `MD_LegInit(&mdLegs[0], "L", &hfdcan1, {10..60})`, `MD_LegInit(&mdLegs[1], "R", &hfdcan2, {70..120})`.
- **Result: both legs run in parallel at 1 kHz, 0 failed cycles with retry** (log `~/nucleo_two_legs.txt`). 12/12 motors pinged, enabled and disabled; TEC / REC / txFail / rxLost = 0 on both legs.

  | Step | Full update, both legs (avg / p99 / max µs) | Left done avg | Right done avg | Failed |
  |---|---|---|---|---|
  | SEQ FULL 1 kHz | 1343 / 1380 / 3680 | – | – | 0 (742 Hz) |
  | PIPE FULL 1 kHz | 912 / 950 / 945 | 845 | 886 | 0 |
  | PIPE PVT 1 kHz | 799 / 840 / 832 | 743 | 775 | 28 |
  | PIPE TAU 1 kHz | 689 / 730 / 730 | 643 | 667 | 0 |
  | PIPE FULL free-run | 912 / 950 / 946 | 845 | 886 | 0 (1093 Hz) |
  | PIPE PVT free-run | 801 / 840 / 832 | 746 | 776 | 39 (1225 Hz) |
  | PIPE TAU free-run | 689 / 730 / 731 | 643 | 667 | 1 (1444 Hz) |
  | PIPE FULL retry 1 kHz | 914 / 950 / 948 | 846 | 886 | **0** (no retries needed) |
  | PIPE PVT retry 1 kHz | 802 / 840 / 898 | 744 | 776 | **0** (35 retries, all answered) |
  | PIPE TAU retry 1 kHz | 691 / 730 / 732 | 644 | 667 | **0** |
  | PIPE PVT retry free-run | 801 / 840 / 910 | 750 | 774 | **0** (1244 Hz) |

- Findings:
  - Parallel works: 12 motors take about as long as 6 on one bus (PVT about 800 µs vs 750 µs).
  - **Right leg starts about 37 µs late** (first reply R70 256 µs vs L10 219 µs): all 6 left commands are queued before the right ones, about 6 µs per `HAL_FDCAN_AddMessageToTxFifoQ` at `-O0`. Fix: interleave the queueing L/R.
  - **About 25 µs CPU overhead per cycle** (full update 912 vs right done 886). Fix: build the motor code with `-O2`.
  - **FULL has no room for a retry at 1 kHz** (p99 950 µs, a retry costs about 200 µs). PVT (p99 840, retried max 898) and TAU (730) fit. → For the 1 kHz control loop use PVT or TAU, and write kp/kd only when the gains change.
  - Without retry, PVT now also misses on left motors 20 and 30 (queue positions 2/3), not just 4/5. Running both buses shifts the timing slightly. The retry covers it; watch it in the soak test.
- **Repeat run** (log `~/nucleo_two_legs_2.txt`): timings identical within 1–2 µs (FULL 912, PVT 799, TAU 689 µs avg). The PVT miss rate without retry varies between runs (54 vs 28 failed in 3 s); with retry 0 failed again (51 retries, worst reply 910 µs). Bus errors 0. The two-leg timings are a reliable baseline.
