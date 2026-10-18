# FDCanUdp — progress notes

Port of `Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino` to the NUCLEO-H755ZI-Q.
Last updated 2026-10-18.

## Target

**Control 2 legs at ≥ 1000 Hz:** one leg per CAN FD network, 6 MD drivers per leg.
- Left leg on FDCAN1, IDs 10–60.
- Right leg on FDCAN2, IDs to be confirmed.

The two FDCAN controllers run in parallel in hardware. If both buses are queued at the start of a cycle, the cycle takes as long as one bus (about 850 µs with FULL frames, 650 µs with TAU), not the sum.

## Status

- **Done, works on hardware:** CM4 drives 6 MAB MD drivers (IDs 10–60) over FDCAN1 at 1 Mbps / 8 Mbps BRS. It pings them, enables them in impedance mode with all gains and targets at zero, and runs the full Portenta benchmark (7 steps × 3 s). Then it disables them.
- **Results match the Portenta** within a few µs (see the table below). The bus is the bottleneck, not the MCU.
- **Open issue:** the PVT (20 B) profile loses replies from motors 40 and 50 (details below).
- **In progress:** FDCAN2 for the second leg. The `.ioc` is configured (see the FDCAN2 settings below); code not regenerated yet.
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

## Open issue: PVT misses on motors 40 and 50

- Only the PVT profile (20 B), only motors 40 and 50 (fw 2.4.0 / 2.4.1). FULL and TAU have zero or almost zero misses. The Portenta shows the same.
- Each miss is a 2 ms timeout: the reply never arrives. The likely cause is the "MD drops a frame right after a reply" behaviour, hit by the PVT timing.
- Worst case seen: 15 consecutive missed cycles on motor 50 (≈ 11 ms without an update), free-running.
- Ideas: resend to the missing motors within the same cycle (about 250 µs left at 1 kHz); use FULL or TAU for control; split the motors across FDCAN1 and FDCAN2.

## Next steps

1. Regenerate the code with FDCAN2. Then check that the SMPS define is still in both `.cproject` files and that the FDCAN1 USER CODE survived.
2. Add FDCAN2's startup code in `fdcan.c` USER CODE (transmitter delay compensation, accept-all filter, start), as for FDCAN1.
3. Make `mab_md` work on two buses (a bus handle per motor or per leg), then run the benchmark on both legs in parallel at 1 kHz.
4. Fix the PVT misses (in-cycle retry), or decide to use FULL/TAU only.
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
- Code not regenerated yet.
