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
- **Both legs run in parallel at 1 kHz** (2026-10-19): 12 motors, 0 failed cycles with retry. With interleaved queueing and `-O2`: PVT 756 µs, TAU 652 µs, FULL 863 µs per full update. Next: the soak test.
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

## Benchmark terms

Each benchmark step name has three parts, for example `PIPE PVT 1000 us` or `PIPE FULL free-run`:

1. **How the commands are sent**
   - **SEQ** (sequential): send to motor 10, wait for its reply, then motor 20, and so on. Slow (about 1230 µs per update); only a baseline.
   - **PIPE** (pipelined): put all commands into the transmit queue at once, then collect the replies as they arrive. The bus is never idle. With two legs, both buses run at the same time.
2. **What each command contains** (a `0x40` register write)

   | Profile | Contents | Frame size |
   |---|---|---|
   | **FULL** | target position, velocity, torque + kp, kd | 32 bytes |
   | **PVT** | target position, velocity, torque (kp/kd keep their last written values) | 20 bytes |
   | **TAU** | target torque only | 8 bytes |

   Every motor answers each command with its 24-byte state (position, velocity, torque, temperature, status), whatever the profile.
3. **How often cycles run**
   - **1000 us**: a new cycle starts exactly every 1 ms (1 kHz), as the control loop will run.
   - **free-run**: the next cycle starts as soon as the previous one ends; shows the highest rate the bus allows.

Plus **retry**: if a motor's reply is missing, its command is sent once more in the same cycle (needed because an MD does not resend a reply that loses arbitration).

Columns in the results: **full update** = time for one complete cycle (all motors commanded and answered); **failed** = cycles where at least one reply never arrived; **over-period** = cycles longer than the 1 ms period; **done** (two-leg runs) = time from cycle start until that leg's last reply; **lat** = time from cycle start until that motor's reply.

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

## UDP link to the PC (CM7) — design, 2026-10-19

**Split:** CM4 runs the 1 kHz FDCAN loop for both legs (already done). CM7 runs Ethernet + LwIP + UDP. They exchange commands and states through mailboxes in SRAM4.

```
PC (WBC) ──UDP cmd──▶ CM7 ──cmd mailbox──▶ CM4 ──FDCAN──▶ 12 motors
PC (WBC) ◀─UDP state─ CM7 ◀─state mailbox─ CM4 ◀─FDCAN── 12 motors
```

**Decisions:**
- PC whole-body controller (WBC) runs at **500 Hz** (if fast enough) or **250 Hz**. CM4 stays at **1 kHz** and uses the latest command (each command is applied 2 or 4 times).
- Motor command: **PVT first**, PVT + kp/kd as plan B. CM4 sends a FULL frame to a motor only when its kp/kd changed, otherwise PVT.
- **PC connected directly** to the Nucleo's RJ45 (no switch). Static IPs: Nucleo `192.168.10.10`, PC `192.168.10.1`, netmask `255.255.255.0`. UDP port **9999** on the Nucleo; the reply goes to the sender's IP and port.
- **Own packet format** for 2 legs × 6 motors (little-endian, packed). Motor order: L 10, 20, 30, 40, 50, 60, then R 70, 80, 90, 100, 110, 120.
- CM7 answers **every command packet with one state packet** (latest states from CM4).
- Command timeout on CM4: if no new command arrives for **20 ms** (5 periods at 250 Hz), switch to a safe action. *Safe action still to decide.*

**Command packet, PC → Nucleo (256 bytes)**

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | magic `0x31434257` ("WBC1") |
| 4 | u16 | version = 1 |
| 6 | u16 | flags: bit0 ENABLE (motors on/off), bit1 GAINS_VALID (use kp/kd) |
| 8 | u32 | seq (incremented by the PC) |
| 12 | u32 | pc_time_us |
| 16 | 12 × {f32 pos, f32 vel, f32 tau, f32 kp, f32 kd} | per motor, 20 bytes each |

**State packet, Nucleo → PC (224 bytes)**

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | magic `0x31545357` ("WST1") |
| 4 | u16 | version = 1 |
| 6 | u16 | status: bit0 enabled, bit1 command timeout, bit2 motor fault |
| 8 | u32 | seq_echo (last command seq applied by CM4) |
| 12 | u32 | stm_time_us |
| 16 | u32 | cm4_cycle (CM4 cycle counter) |
| 20 | u16 | cycle_us (last full CAN update) |
| 22 | u16 | cmd_age_us (age of the applied command) |
| 24 | 8 bytes | reserved |
| 32 | 12 × {f32 pos, f32 vel, f32 tau, u16 quickStatus, u8 temperature, u8 flags} | per motor, 16 bytes each; flags: bit0 online, bit1 enabled, bit2 reply missed this cycle |

**Memory:**
- D2 SRAM3 `0x30040000–0x30047FFF` (32 KB) reserved for the Ethernet DMA descriptors and RX buffers. CM4's RAM was shrunk from 288 KB to 256 KB to free it (CM4 uses about 3 KB).
- LwIP heap in CM7's normal RAM (AXI SRAM, which the ETH DMA can reach). CM7 D-cache stays **off**; if it is ever turned on, the ETH buffers and SRAM4 need non-cacheable MPU regions.
- SRAM4: log ring buffer at `0x38000000` (8 KB); command and state mailboxes to be placed after it.

**Steps:**
1. CubeMX: ETH (RMII, LAN8742, PHY address 0) + LWIP on **CM7**, static IP, no DHCP; ETH pins moved from CM4 to CM7; linker sections for the descriptors in SRAM3. Test: `ping 192.168.10.10`.
2. UDP echo on CM7 + Python test client on the PC; measure the round trip. Make the log forwarding non-blocking (it currently blocks CM7 for up to 0.4 s per window printout).
3. Mailboxes CM7 ↔ CM4 with seqlock (no half-written snapshots) and freshness checks.
4. CM4 control loop replacing the benchmark: latest command, PVT/FULL + retry, timeout → safe action.
5. Full loop PC → motors → PC at 250/500 Hz, latency measurement, soak test through UDP.

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
3. ~~Cut the cycle time~~ (done 2026-10-19: interleaved queueing + `-O2`, 37–49 µs saved per cycle; PVT 756 µs, TAU 652 µs, FULL 863 µs for both legs).
4. Soak test both legs: PVT + retry at 1 kHz for 10–30 min (see "Is PVT control ready?").
4. ~~Fix the PVT misses~~: solved with ascending-ID order + in-cycle retry (2026-10-18). Use the retry in the 2-leg control loop.
5. UDP link to the PC on CM7 + command/state mailboxes CM7 ↔ CM4: see "UDP link to the PC (CM7) — design" (own format, not the Portenta's).
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
- **Repeat run** (log was in `~/nucleo_two_legs_2.txt`, since overwritten by the `-O2` run): timings identical within 1–2 µs (FULL 912, PVT 799, TAU 689 µs avg). The PVT miss rate without retry varies between runs (54 vs 28 failed in 3 s); with retry 0 failed again (51 retries, worst reply 910 µs). Bus errors 0. The two-leg timings are a reliable baseline.
- Committed as `1bd3efc`.

### 2026-10-18 — cutting the two-leg cycle time
- `updatePipelined()` now queues the legs interleaved (L10, R70, L20, R80, …) instead of all left then all right. (Log not saved to a file; numbers from the terminal output.)

  | | Before (all L, then all R) | After (interleaved) |
  |---|---|---|
  | First reply R70 vs L10 (FULL) | 257 vs 219 µs (+38) | 231 vs 219 µs (+12) |
  | Right leg done (FULL) | 886 µs | 858 µs |
  | Full update FULL (avg / p99) | 912 / 950 | **891 / 920** |
  | Full update PVT (avg / p99) | 799 / 840 | **783 / 820** |
  | Full update TAU (avg / p99) | 689 / 730 | **679 / 710** |
  | PVT retry 1 kHz (avg / max) | 802 / 910 | 786 / 902 |
  | Failed cycles with retry | 0 | 0 |

  - The remaining 12 µs offset is one `HAL_FDCAN_AddMessageToTxFifoQ` call at `-O0` (about 12 µs each, so about 140 µs of CPU to queue 12 frames).
  - About 33 µs of the cycle is CPU only (full update 891 vs right done 858): flushing both RX FIFOs before the cycle plus the bookkeeping after it.
  - Both are CPU-bound → next: `-O2`.
- **CM4 Debug configuration switched to `-O2`** (Properties → C/C++ Build → Settings → MCU GCC Compiler → Optimization). Switch back to `-O0` for single-step debugging sessions. Log `~/nucleo_two_legs_2.txt` (verified against the table below).

  | | Baseline (`-O0`, L then R) | Interleave (`-O0`) | Interleave + `-O2` |
  |---|---|---|---|
  | First reply R70 vs L10 (FULL) | 257 vs 219 µs | 231 vs 219 µs | **217 vs 211 µs** |
  | CPU overhead (full update − slower leg) | 26 µs | 33 µs | **20 µs** |
  | Full update FULL (avg / p99) | 912 / 950 | 891 / 920 | **863 / 890** |
  | Full update PVT (avg / p99) | 799 / 840 | 783 / 820 | **756 / 790** |
  | Full update TAU (avg / p99) | 689 / 730 | 679 / 710 | **652 / 690** |
  | PVT retry 1 kHz (avg / max) | 802 / 910 | 786 / 902 | **758 / 821** |
  | SEQ FULL | 1344 µs (742 Hz) | 1344 µs | **1230 µs (811 Hz)** |
  | Failed cycles with retry | 0 | 0 | **0** |

  - 37–49 µs saved per cycle in total. 12 motors with PVT (756 µs) now take about as long as 6 motors on one bus did (750 µs).
  - PVT retry worst case 821 µs → about 180 µs margin at 1 kHz. FULL p99 890 µs → about 110 µs margin (no retries needed in this run).
  - **Without retry the misses moved:** left leg 0 misses, right leg motors 80/90/100 up to 29 cycles in a row; TAU had 2 misses for the first time. The faster code shifted which replies collide. The miss pattern depends on exact timing and can change with any code change → **the retry is mandatory**; the soak test must look for double misses (retry lost too).

### 2026-10-19 — UDP link design
- Decided: CM4 = FDCAN (1 kHz), CM7 = Ethernet/UDP. PC WBC at 500 or 250 Hz, PVT first (FULL when kp/kd change), direct RJ45 link, own packet format (see "UDP link to the PC (CM7) — design").
- CM4 RAM shrunk from 288 KB to 256 KB (`CM4/STM32H755ZITX_FLASH.ld`) so D2 SRAM3 is free for the Ethernet DMA. CM4's stack top moved from 0x10048000 to 0x10040000.
- Next: CubeMX ETH + LWIP on CM7.

### 2026-10-19 — ETH + LwIP generated on CM7
- CubeMX: ETH (RMII) + LWIP 2.2.1 on CM7, static IP 192.168.10.10/24, no DHCP, LAN8742 PHY. ETH pins moved from CM4 to CM7 (CM4 `gpio.c` no longer touches them).
  - CubeMX IP fields need 3 digits per number (`192.168.010.010`). The generated `lwip.c` uses plain decimals (no octal trap).
  - The gateway was generated as 192.168.101.0 (typo); fixed to 192.168.10.1 in the `.ioc` and `lwip.c`.
- **Ethernet DMA memory (CM7 linker script):** new region `RAM_ETH` = D2 SRAM3 `0x30040000`, 32 KB, section `.eth_dma`. Verified in the build:

  | Object | Address | Size |
  |---|---|---|
  | RX descriptors (`.RxDescripSection`) | `0x30040000` | 96 B |
  | TX descriptors (`.TxDescripSection`) | `0x30040060` | 96 B |
  | RX buffer pool (`.Rx_PoolSection`, 12 × 1.5 KB) | `0x300400C0` | 18.8 KB |
  | LwIP heap (`LWIP_RAM_HEAP_POINTER`, `MEM_SIZE` 8192) | `0x30045000` | 8 KB |

  - The ETH descriptor addresses in the CubeMX ETH panel (0x30000000/0x30000080) apply only to IAR/Keil; with GCC the linker script decides.
  - **CubeMX defaulted `LWIP_RAM_HEAP_POINTER` to `0x30004000`** = inside CM4's RAM (D2 SRAM1). Moved to `0x30045000` in `lwipopts.h` and the `.ioc`. A linker `ASSERT` fails the build if the RX pool grows into the heap. **Check this after every regenerate.**
- CM7 `main.c`: `MX_LWIP_Process()` in the main loop (USER CODE 3), before `IPC_LogDrain()`.
- Both cores build (CM7 116 files with LwIP; CM4 `-O2` setting survived the regenerate).
- Next: CubeIDE must pick up the new LWIP/Middlewares files (close/reopen or re-import CM7), then flash and `ping 192.168.10.10` from the PC.
- Regenerated twice more (PB0 / `LED1_Green` re-enabled on CM7). **Lesson:** while CubeMX has the `.ioc` open, hand edits to the `.ioc` are overwritten on save, and the GUI input masks invite typos (the gateway came out as 192.168.101.0, then 192.160.10.1; the heap pointer reverted to 0x30004000).
- **Fix: the critical values now live in USER CODE blocks**, which survive every regenerate:
  - `CM7/LWIP/Target/lwipopts.h`, last `USER CODE 1` block: `#undef`/`#define LWIP_RAM_HEAP_POINTER 0x30045000`, `MEM_SIZE 8192`.
  - `CM7/LWIP/App/lwip.c`, `USER CODE IP_ADDRESSES`: IP 192.168.10.10, netmask 255.255.255.0, gateway 192.168.10.1 (overrides whatever CubeMX generated).
  - Verified in the CM7 build: only `0x30045000` is referenced, never `0x30004000`.
- `.gitignore` fixed: the old rule `.settings/language.settings.xml` only matched at the repo root, so both projects' `language.settings.xml` were tracked and changed on every build. Now `**/.settings/language.settings.xml` and `**/.settings/*.prefs`; the two files were untracked (`git rm --cached`, still on disk). `Portenta_H7/` stays untracked (reference only).
- Committed as `77ce7e1`.

### 2026-10-19 — first ping, network status LED
- **Ping works** (`ping 192.168.10.10` from the PC). Pitfalls on the way:
  - The PC's wired port (`enp1s0`) had been given `192.168.10.10`, the Nucleo's address, so the first "replies" came from the PC itself (0.02–0.07 ms, TTL 64; `ip route get` showed `local … dev lo`). PC must be **192.168.10.1/24**. A real reply from the Nucleo has TTL 255.
  - After fixing the PC address, ARP got no answer until the board was reset; afterwards it worked. Cause unclear (probably the PC port was still being reconfigured); watch for it after fresh flashes.
  - The RJ45 link LED / "link up 100 Mbit/s" only proves the PHY is powered: the LAN8742 negotiates on its own, even if the STM32 never configures the MAC.
- **LED_UDP = LD2 on PE1** (yellow on this board), configured on CM7 in CubeMX. `udpLedUpdate()` in CM7 `main.c`:
  - off = Ethernet link down;
  - slow blink 1 Hz = link up, no PC traffic;
  - fast blink 10 Hz = unicast packets from the PC in the last 200 ms (MAC counter `ETH->MMCRUPGR`). After the UDP step this will mean valid command packets.
- `CM4/.settings/stm32cubeide.project.prefs` untracked (committed before the ignore rule existed).
- Verified on the board: slow blink when idle, fast blink while pinging.
- Committed as `db87184`.

### 2026-10-19 — UDP server on CM7 (design step 2)
- `Common/Inc/wbc_protocol.h`: the packet layout from the design (command 256 B, state 224 B) as packed C structs with `_Static_assert` size checks. Shared by CM7, CM4 and the PC client.
- `CM7/Core/Src/udp_server.c`: LwIP raw API (NO_SYS), port 9999. The receive callback runs inside `MX_LWIP_Process()`, checks length / magic / version, counts sequence gaps, and answers each valid command at once with one state packet to the sender. **States are placeholders in this step** (status bit `NO_CM4_DATA`, seq echoed, CM7 µs time); step 3 fills them from CM4.
- `CM7/Core/Src/cm7_log.c`: **non-blocking serial log.** USART3 has no IRQ on CM7, so `CM7_LogPump()` (every main-loop pass) writes at most one byte when TXE is set. Sources: CM7's own `CM7_LOG()` queue first, then the CM4 log via the new `IPC_LogRead()`. Replaces the blocking `IPC_LogDrain()`, which stalled CM7 for up to 0.4 s.
- CM7 `main.c`: `UDP_Server_Init()` after `MX_LWIP_Init()`, UDP stats every 5 s while commands arrive, LED fast blink now means **valid command packets** (ping alone gives slow blink).
- `pc/wbc_udp_test.py`: sends commands at a fixed rate (default 500 Hz, 10 s), matches replies by seq, reports lost / late / bad, RTT avg/min/p50/p99/max and the PC's own send-period jitter. Never sets ENABLE unless `--enable`. `--rt` uses SCHED_FIFO (root). Tested against a local fake server.
- First test on the board (`sudo python3 pc/wbc_udp_test.py --rate 250 --duration 10 --rt`): **0 of 2500 replies, and ping failed too**, although the log showed `[UDP] listening` and CM7's main loop was alive (still forwarding the CM4 log). Link up 100 Mbit/s, PC ARP entry STALE.
  - Same symptom as the first ping (dead after flashing, worked after RESET). Two candidate causes: the debugger start after flashing leaves ETH/PHY in a bad state, or the RX DMA runs out of buffers after some traffic and stops (known weak spot of the CubeMX LwIP driver on the H7).
  - Added `netStatusUpdate()` to CM7 `main.c`: a `[NET]` log line on every link change and every 5 s with MAC receiver on, RX DMA running, DMA flags RBU (no free RX buffer) / RPS (RX stopped) / FBE (bus error), MAC counters (unicast received, CRC errors, transmitted) and the HAL ETH state/error.
  - `[NET]` result: receiver and RX DMA running, no DMA errors (RBU/RPS/FBE 0), no CRC errors; pings were received and answered (rxUni/txGood rose). **The log stops exactly when the UDP test starts**, and a live serial capture afterwards got 0 bytes in 16 s (normally a `[NET]` line every 5 s), with no debugger attached. → **CM7 crashes or hangs on the first UDP command packet**; pings alone are fine.
  - Added fault reporting: CM7's HardFault/MemManage/BusFault/UsageFault handlers call `CM7_FaultDump()` (cm7_log.c), which prints the fault status registers (CFSR, HFSR, MMFAR, BFAR) and the stacked PC/LR with blocking UART writes. It finds the exception frame itself (Thumb bit in xPSR, PC in flash), because the handler's prologue size depends on the compiler.
  - Fault caught (`[CM7] HardFault PC=0x08008502 LR=0x080023E3 CFSR=0x00000400 HFSR=0x40000000`): CFSR 0x400 = **imprecise bus error**, inside `HAL_ETH_RxLinkCallback()` (ethernetif.c), reached from `HAL_ETH_ReadData()`. The callback computes the pbuf from `buff − offsetof(RxBuff_t, buff)`; with `buff = BackupAddr0 = 0` (RX pool allocation failed) that writes to 0xFFFFFFE0.
  - **Root cause found in the CubeMX NO_SYS port: TX descriptors are never released.** `low_level_output()` uses the blocking `HAL_ETH_Transmit()`, but nothing calls `HAL_ETH_ReleaseTxPacket()`. The HAL keeps `PacketAddress[]` set, so after `ETH_TX_DESC_CNT` = 4 frames every send fails with `HAL_ETH_ERROR_BUSY`. Explains: `txGood` stuck at 4–5, the first few pings answered then nothing, "works after RESET" briefly, the PC receiving nothing without errors.
  - **Fix (USER CODE only, survives regeneration):**
    - `lwip.c` USER CODE 0/3: wrap `gnetif.linkoutput`: call the generated `low_level_output()`, then `HAL_ETH_ReleaseTxPacket(&heth)`.
    - `ethernetif.c` `HAL_ETH_TxFreeCallback`: no `pbuf_free()` anymore. `low_level_output()` takes no reference and lwIP frees the pbuf after the (blocking) send; freeing it in the callback would free every TX pbuf twice once the release works.
  - The RX-pool exhaustion behind the crash is not explained yet by the TX bug alone; check whether the crash is gone after the TX fix (fault reporting stays in).
  - **Result after the TX fix:** ping is now reliable (5/5, 0.19 ms; before: a few replies, then nothing). **UDP still crashes CM7 after 3 replies** (RTT about 220 µs): same HardFault, same place (`HAL_ETH_RxLinkCallback`, `buff = BackupAddr0 = 0`, CFSR 0x400). So there is a second, RX-side problem that shows with UDP (or with a high packet rate).

### Where to continue (stopped 2026-10-19 ~02:00)
- **Board state when stopped:** CM7 crashed (HardFault); needs a RESET before it answers on the network.
- **Uncommitted** since `db87184`: UDP server, `wbc_protocol.h`, non-blocking CM7 log, `[NET]` status line, fault reporting, the TX-release fix, `pc/wbc_udp_test.py`, `pc/serial_log.sh`.
- **Next experiments** (each needs a reset; asked the user whether I may reset via `STM32_Programmer_CLI` over SWD — no answer yet; a reset re-runs the CM4 benchmark, which enables the 12 motors at zero gains):
  1. Fast ping (`ping -i 0.004`, 250/s): crash too → it is the packet rate, not UDP.
  2. UDP packets of the wrong size to port 9999 (dropped without reply): crash → RX path; no crash → the reply path.
  3. UDP at 5/s: separates packet count from rate.
- Ideas to check: RX pool (12 buffers) consumption per packet (add counters in `HAL_ETH_RxAllocateCallback` / `pbuf_free_custom`), LwIP heap use by the reply (`MEM_SIZE` 8192 at 0x30045000), whether RX descriptors get corrupted (dump `heth.RxDescList` + `BackupAddr0` of all 4 descriptors in the fault handler), known CubeMX H7 issues around `ETH_UpdateDescriptor()` after `RX_ALLOC_ERROR`.
- Still open from the design: what CM4 does when PC commands stop (damping / hold / disable).
- Tools: `pc/serial_log.sh <name>` (log to `~/nucleo_<name>_MMDD_HHMM.txt`), `python3 pc/wbc_udp_test.py --rate 250|500 --duration N [--rt with sudo]`. I can also read the serial port directly (`stty -F /dev/ttyACM0 115200 raw; timeout N cat /dev/ttyACM0`) while running tests.

### 2026-10-19 (later) — resumed
- Motors are **not needed** for the UDP debugging (CM7 only). With the motor power off, CM4 finds 0 motors and enables nothing, so the board can be reset freely and the log stays quiet.
- **Pitfall:** after power-on, the PC's wired port `enp1s0` was down (no carrier, no IP). Linux then routed 192.168.10.10 through **Wi-Fi via 192.168.1.1**, and another machine on that network answered pings (TTL 60, 1–160 ms) while the Nucleo log said `link DOWN`. Always check `ip -br addr show enp1s0` → `UP 192.168.10.1/24` and `ip route get 192.168.10.10` → `dev enp1s0`.
- `pc/wbc_udp_test.py` now refuses to run unless the target is reached from a local 192.168.10.x address (prevents sending commands to a wrong host over Wi-Fi).

### 2026-10-19 (afternoon) — UDP crash and stalls found and fixed; 5-minute soak clean
Worked alone (user away; allowed to reset and flash via `STM32_Programmer_CLI` over SWD, motors powered off so resets are harmless). Every step: build CM7 by hand, flash only CM7's bank, reset, run tests while reading the serial port.

**Root cause 1 — HardFault in `HAL_ETH_RxLinkCallback`: D-cache invalidate on an uninitialised cache.**
- All crashes (CFSR 0x400, imprecise bus error) were inside `SCB_InvalidateDCache_by_Addr()` called by the RX link callback. Proven with `__DSB()` after every write in the callback: all passed, and the fault still fired at the `SCB->DCIMVAC` store itself.
- CM7 runs with the D-cache **off** and never initialises it (normally `SCB_EnableDCache()` invalidates it first). Invalidate-by-address still looks up the cache tag RAM, which holds random data (with ECC) after power-up → bus error, at random moments depending on the buffer addresses.
- **Fix** (`ethernetif.c`, USER CODE RxLinkCallback): invalidate only if `SCB->CCR & SCB_CCR_DC_Msk` (D-cache enabled). With the cache off the CPU reads DMA data directly from SRAM.
- Hypotheses ruled out on the way: RX pool leak (pool stayed 8/12 free), SRAM ECC on uninitialised SRAM3 (zero-filling did not help), missing `D2SRAM3EN` (setting it did not help). Both of those are kept anyway as good practice: CM7 now sets `__HAL_RCC_D2SRAM3_CLK_ENABLE()` and zero-fills SRAM3 with 32-bit writes in USER CODE SysInit, before `MX_LWIP_Init()`.

**Correction of yesterday's TX diagnosis.** The descriptors were *not* stuck after 4 frames: the HAL's BUSY check uses `PacketAddress[]`, which is always NULL with the blocking send. The early ping failures were root cause 1; ping passing 5/5 after the "TX fix" was luck.

**Root cause 2 — growing stalls (lost / late replies after 10–20 s at 500 Hz).**
- Measured per main-loop pass: one transmit grew from 20 µs to 2.5 ms, then stalls of 20 ms and 5–10 s; RX descriptors overflowed (`RBU 1`), the board missed thousands of commands (`seqGaps`).
- Live read of `heth.TxDescList` over SWD: `BuffersInUse` grew by +1 per frame (2027 → 4053 → 6087 …), `PacketAddress[]` all NULL.
- Cause: the blocking `HAL_ETH_Transmit()` never sets `TxDescList.CurrentPacketAddress` (only `HAL_ETH_Transmit_IT()` does), so `PacketAddress[]` stays NULL and `HAL_ETH_ReleaseTxPacket()` can never release anything, while `ETH_Prepare_Tx_Descriptors()` adds +1 per frame. My release wrapper called `ReleaseTxPacket()` after every frame, which loops `BuffersInUse` times.
- **Fix** (`lwip.c`, USER CODE 0/3): the linkoutput wrapper now resets `heth.TxDescList.BuffersInUse = 0` after the blocking send instead of calling `ReleaseTxPacket()`. `HAL_ETH_TxFreeCallback` stays a no-op.

**Robustness / speed additions:**
- RX DMA self-heal (`lwip.c` USER CODE 4_2): if RBU is set, clear it and re-write the RX tail pointer (`lwipRxDmaKicks` counts it; 0 so far).
- CM7 I-cache enabled (`SCB_EnableICache()` in USER CODE Init); D-cache stays off. Worst loop pass 353 → 225 µs.
- One compact `[NET]` status line every 5 s: link, MAC counters, RBU/kicks, RX pool, worst loop/rx/timer/tx times, HAL error.
- Kept: fault reporting (`CM7_FaultDump`), RX buffer guard in the link callback (counts and skips buffers outside the RX pool; 0 so far).

**Results (motors off, PC without real-time priority):**

| Test | Result |
|---|---|
| Ping 250/s | 500/500, 0.10 ms |
| UDP 500 Hz, 30 s | 15 000/15 000, 0 lost, RTT avg 216 µs, p99 299 µs, max 732 µs |
| **UDP 500 Hz, 5 min** | **150 000/150 000, 0 lost, 0 late**, RTT avg 226 µs, p99 330 µs |
| UDP 250 Hz, 3 min | 44 999/45 000 (1 late reply: PC-side stall of 9.4 ms; the board answered all 45 000) |
| Board over 195 000 packets | 0 seqGaps, 0 RBU, pool 8/12 steady, worst loop pass 300 µs, no faults |

Board-side timings now flat: one send 30 µs, RX processing 76 µs, LwIP timers + PHY link check 144 µs (every 100 ms).

### Where to continue
- **Uncommitted** since `db87184`: everything from the UDP step (server, protocol header, CM7 log, fault reporting, the two Ethernet fixes, diagnostics) plus `pc/` scripts. Verified stable — ready to commit.
- The board currently runs my hand-built CM7 image; the sources in the repo are identical, so a normal CubeIDE build + Run gives the same firmware (CubeIDE builds CM7 at `-O0`, as I did).
- Next per the design: **step 3**, the CM7 ↔ CM4 mailboxes in SRAM4 (seqlock), then step 4, the CM4 control loop. Still needs the user's decision: what CM4 does when PC commands stop (damping / hold / disable).
- For PC-side timing in real use, run the WBC process with real-time priority (`--rt` / SCHED_FIFO); the one late reply in the soak came from a 9 ms PC stall.
