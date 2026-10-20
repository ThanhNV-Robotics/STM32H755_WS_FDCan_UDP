# STM32H755 FDCAN ↔ UDP bridge for a two-legged robot

Firmware for a **NUCLEO-H755ZI-Q** that connects a PC running a whole-body controller (WBC) to the **12 MAB MD80 motor drivers** of a robot's two legs.

- **CM4 core:** drives both legs over two **CAN FD** buses at **1 kHz**.
- **CM7 core:** talks to the PC over **Ethernet / UDP** at **250–500 Hz** (1 kHz as a goal).

It is a port and extension of an earlier Arduino Portenta H7 prototype (`FDCanUdp/Portenta_H7/`, reference only).

> This README describes the **target system**. The [Status](#status) table shows what is implemented and measured today. The detailed development history, measurements and pitfalls are in [`FDCanUdp/NOTES.md`](FDCanUdp/NOTES.md).

---

## Architecture

```
                 Ethernet 100 Mbit/s, UDP :9999                 SRAM4 mailboxes
 ┌───────────┐   command 256 B  ──────────▶   ┌─────────────────────┐  ──────────▶  ┌─────────────────────────┐
 │  PC (WBC) │                                │ CM7  400 MHz        │               │ CM4  200 MHz            │
 │ 250/500 Hz│   ◀──────────  state 224 B     │ LwIP, UDP server,   │  ◀──────────  │ 1 kHz control loop      │
 └───────────┘                                │ serial log, LEDs    │               │ MD80 protocol, retry    │
   192.168.10.1                               └─────────────────────┘               └──────┬───────────┬──────┘
                                                192.168.10.10                         FDCAN1│           │FDCAN2
                                                                                  1 / 8 Mbps│           │1 / 8 Mbps
                                                                                     ┌──────▼─────┐ ┌───▼────────┐
                                                                                     │ Left leg   │ │ Right leg  │
                                                                                     │ IDs 10–60  │ │ IDs 70–120 │
                                                                                     └────────────┘ └────────────┘
```

### Responsibilities

| Part | Runs on | Does |
|---|---|---|
| **Whole-body controller** | PC (Linux) | Computes per-joint targets at 250 or 500 Hz. Sends one command packet per cycle, receives one state packet back. |
| **Network bridge** | CM7 (Cortex-M7, 400 MHz) | Ethernet + LwIP (no RTOS). Validates command packets, puts them in the command mailbox, answers each with the latest motor states. Forwards the serial log and drives the network status LED. |
| **Motor control loop** | CM4 (Cortex-M4, 200 MHz) | Fixed 1 kHz cycle. Takes the latest command, sends it to all 12 motors on both buses in parallel, collects the 12 replies (with an in-cycle retry), and writes the states to the state mailbox. Enforces the safety rules (boot disabled, command timeout). |
| **Shared memory** | SRAM4 (D3 domain) | Log ring buffer (CM4 → CM7) and the command / state mailboxes. Lock-free: one writer per area, sequence counters against torn reads. |

The two cores are deliberately decoupled. Network traffic, which is bursty and unpredictable, can never disturb the timing of the CAN cycle, and a slow or missing PC can't stall the motors. The CM4 loop keeps running at 1 kHz and applies the latest command (or the safe action, see [Safety](#safety)).

### One control cycle (1 ms, CM4)

1. Read the latest command from the mailbox (consistent snapshot).
2. Queue one `0x40` register-write frame per motor on **both** controllers, interleaved (L10, R70, L20, R80, …), in ascending CAN ID order per bus.
3. Collect the 12 replies from both RX FIFOs. If a reply is missing, resend that one command once, in the same cycle.
4. Write the 12 states (position, velocity, torque, temperature, status, flags) and the cycle statistics to the state mailbox.

---

## Hardware and wiring

### Components

| Item | Details |
|---|---|
| Board | **NUCLEO-H755ZI-Q** (STM32H755ZIT6, dual core), stock configuration |
| Motor drivers | 12 × **MAB MD80**, 6 per leg, mixed firmware (v2.4.0 / v2.4.1 / v2.5.4) |
| CAN FD transceivers | 2 × external CAN FD transceiver capable of 8 Mbps data phase (the Nucleo has none) |
| PC link | Direct Ethernet cable to the Nucleo's RJ45 (no switch) |
| Debug / log | ST-LINK V3 on the Nucleo (USB): SWD flashing and a virtual COM port |

### Pin assignment

| Function | Pin(s) | Core | Notes |
|---|---|---|---|
| FDCAN1 RX / TX (left leg) | **PD0** (RX), **PD1** (TX) | CM4 | PD1 → transceiver TXD, PD0 ← transceiver RXD |
| FDCAN2 RX / TX (right leg) | **PB5** (RX), **PB6** (TX) | CM4 | PB6 → transceiver TXD, PB5 ← transceiver RXD |
| Ethernet (RMII, LAN8742 PHY on board) | PA1, PA2, PA7, PC1, PC4, PC5, PB13, PG11, PG13 | CM7 | Nucleo RJ45 |
| Serial log (USART3 → ST-LINK VCP) | PD8 (TX), PD9 (RX) | CM7 | `/dev/ttyACM0`, 115200 8N1 |
| LED LD1 green | PB0 | CM7 (configured), CM4 (driven) | Motor bus status |
| LED LD3 red | PB14 | CM7 (configured), CM4 (driven) | Motor bus error |
| LED LD2 yellow (`LED_UDP`) | PE1 | CM7 | Network status |

### CAN FD bus wiring (per leg)

```
 Nucleo                 CAN FD transceiver               MD80 chain (one leg)
 TX pin  ───────────▶  TXD                       CANH ──┬── MD ── MD ── MD ── MD ── MD ── MD ──┬──
 RX pin  ◀───────────  RXD                       CANL ──┴───────────────────────────────────────┴──
 GND     ────────────  GND                              120 Ω                                  120 Ω
 3.3 V   ────────────  VIO (if present)
 5 V     ────────────  VCC
 GND     ────────────  STB / S / silent pin  (normal mode; EN-type parts: EN to 3.3 V instead)
```

- One transceiver and one bus per leg. Each bus is terminated with **120 Ω at both ends**.
- Common ground between the Nucleo, the transceivers and the motor drivers.
- The transceiver's standby / silent pin must be in **normal mode**. A floating pin puts it in standby: the motors see the traffic, but nothing gets acknowledged.

### Motor IDs

Each bus has its own ID space. IDs must be **ascending in queue order** (see [Design notes](#design-notes)).

| Joint | Left leg (FDCAN1) | Right leg (FDCAN2) |
|---|---|---|
| hip pitch | 10 | 70 |
| hip roll | 20 | 80 |
| hip yaw | 30 | 90 |
| knee pitch | 40 | 100 |
| ankle upper motor | 50 | 110 |
| ankle lower motor | 60 | 120 |

The right leg is assumed to use the same joint order as the left leg (left-leg joint names from `Project_Description.md`).

### Network

| | Address |
|---|---|
| Nucleo | `192.168.10.10/24`, UDP port **9999** |
| PC (wired port) | `192.168.10.1/24` (static, also used as gateway) |
| Reply | Sent to the source IP and port of each command |

---

## Specifications

### Rates and timing

| Item | Target | Measured (see Status) |
|---|---|---|
| CM4 control loop | 1 kHz, fixed | 1 kHz, 0 failed cycles with retry |
| Full update, 12 motors, PVT frames (both legs in parallel) | < 1 ms | avg 758 µs, worst 821 µs |
| Full update, TAU frames | < 1 ms | avg 653 µs, worst 728 µs |
| Full update, FULL frames (PVT + kp/kd) | only when gains change | avg 863 µs, p99 890 µs (no room for a retry at 1 kHz) |
| PC command rate | 500 Hz (or 250 Hz); 1 kHz as a goal | UDP tested at 250 / 500 Hz |
| UDP round trip PC → CM7 → PC | < 1 ms | avg 226 µs, p99 330 µs (500 Hz, 5 min) |
| Command timeout on CM4 | 20 ms (proposed) | – |

### CAN FD

| Parameter | Value |
|---|---|
| Kernel clock | 80 MHz (PLL2Q) |
| Nominal (arbitration) bit rate | **1 Mbps**: prescaler 1, 80 tq (1 + 63 + 16), SJW 16, sample point 80 % |
| Data bit rate (BRS) | **8 Mbps**: prescaler 1, 10 tq (1 + 7 + 2), SJW 2, sample point 80 % |
| Frame format | CAN FD with bit rate switch, standard 11-bit IDs |
| Transmitter delay compensation | On (offset = prescaler × DataTimeSeg1) |
| Automatic retransmission | On (needed when commands lose arbitration to replies) |
| FIFOs (per controller) | RX FIFO0 16 × 64 B, TX FIFO 16 × 64 B, accept-all global filter |
| Message RAM | FDCAN1 words 0–575, FDCAN2 words 576–1151 (offset 576) |

### Motor protocol (MAB MD80)

| Frame | Use | Size |
|---|---|---|
| `0x41` read register | Ping (firmware version `0x802`), config read-back | 8 B |
| `0x40` write register (legacy) | All writes and the cyclic command (firmware 2.4.x ignores `0x42`) | 8–32 B |
| `0xA0` default response | Reply to every `0x40`: quick status, temperature, position, velocity, torque | 24 B |

Cyclic command profiles:

| Profile | Registers | Frame | Use |
|---|---|---|---|
| **PVT** | target position, velocity, torque | 20 B | Default cyclic command |
| **FULL** | PVT + kp, kd (impedance gains) | 32 B | Only for motors whose gains changed |
| **TAU** | target torque | 8 B | Torque-only control (fastest) |

Full protocol reference: [`mab_can_protocol.md`](mab_can_protocol.md).

### Clocks and power (Nucleo-specific, do not change)

| Item | Setting |
|---|---|
| Core supply | **Direct SMPS** (`PWR_DIRECT_SMPS_SUPPLY`, define `USE_PWR_DIRECT_SMPS_SUPPLY`). An LDO setting locks the chip and must never be used on this board. |
| HSE | 8 MHz **bypass** from the ST-LINK MCO |
| CPU clocks | CM7 400 MHz, CM4 200 MHz (PLL1: 8 / 2 × 200 / 2), voltage scale 1 |
| Boot | Dual-core boot sync: CM4 waits until CM7 has configured the clocks |

---

## UDP protocol

Little-endian, packed, defined once in [`FDCanUdp/Common/Inc/wbc_protocol.h`](FDCanUdp/Common/Inc/wbc_protocol.h) (with compile-time size checks) and mirrored in `pc/wbc_udp_test.py`. Motor order in both packets: index 0–5 = left leg 10…60, 6–11 = right leg 70…120.

### Command, PC → Nucleo (256 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | magic `0x31434257` ("WBC1") |
| 4 | u16 | version = 1 |
| 6 | u16 | flags: bit0 `ENABLE` (motors on/off), bit1 `GAINS_VALID` (kp/kd are valid) |
| 8 | u32 | seq (incremented by the PC) |
| 12 | u32 | PC time (µs) |
| 16 | 12 × 20 B | per motor: f32 position (rad), f32 velocity (rad/s), f32 torque (Nm), f32 kp, f32 kd |

### State, Nucleo → PC (224 bytes, one per command)

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | magic `0x31545357` ("WST1") |
| 4 | u16 | version = 1 |
| 6 | u16 | status: bit0 enabled, bit1 command timeout, bit2 motor fault, bit3 no CM4 data |
| 8 | u32 | seq echo (last command applied by CM4) |
| 12 | u32 | Nucleo time (µs) |
| 16 | u32 | CM4 cycle counter |
| 20 | u16 | duration of the last full CAN update (µs) |
| 22 | u16 | age of the applied command (µs) |
| 24 | 8 B | reserved |
| 32 | 12 × 16 B | per motor: f32 position, f32 velocity, f32 torque, u16 quick status, u8 temperature, u8 flags (bit0 online, bit1 enabled, bit2 reply missed this cycle) |

---

## Safety

Target behaviour of the CM4 control loop:

- **Disabled at boot.** Motors are only enabled when the PC sends commands with the `ENABLE` flag set. Clearing the flag disables all motors.
- **Command timeout.** If no new command arrives for 20 ms, CM4 switches to a safe action and sets the `CMD_TIMEOUT` status bit. *The safe action (damping / hold position / disable) is still to be decided.*
- **Motor CAN watchdog.** Every MD80 stops on its own if it receives nothing for 100 ms (firmware 2.4) / 200 ms (firmware 2.5.4), so the loop must never pause longer.
- **Per-motor health.** Missed replies are retried in the same cycle; motors that stay silent are flagged in the state packet.
- **Gains on demand.** kp/kd are only sent (as a FULL frame) when the PC changes them; PVT frames leave them unchanged.

---

## Design notes

Hard-won facts this design relies on (details and measurements in `NOTES.md`):

- **An MD80 does not retransmit a reply that loses arbitration.** Commands must be queued in ascending CAN ID order, and a missing reply is resent once within the cycle. With that, 0 failed cycles at 1 kHz.
- **Both buses run in parallel.** The two FDCAN controllers transmit independently, so 12 motors take about as long as 6 on one bus. Queueing alternates between the legs so neither leg starts late.
- **The CubeMX Ethernet driver needed two fixes** (both in USER CODE blocks, so they survive regeneration):
  - Skip the D-cache invalidate in the RX callback while the cache is off (it caused random imprecise bus faults).
  - Reset `TxDescList.BuffersInUse` after each blocking `HAL_ETH_Transmit()` (the HAL never releases it, which led to growing stalls).
- **Memory map:**
  - D2 SRAM1 + SRAM2 (256 KB) for CM4.
  - D2 SRAM3 `0x30040000` (32 KB) for the Ethernet DMA descriptors, RX buffers and LwIP heap.
  - SRAM4 `0x38000000` for the inter-core log and mailboxes.
- **Firmware 2.4.x replies `0x00` instead of echoing `0x41`** on reads; only the echoed register address is checked.

---

## Repository layout

```
FDCanUdp/
├── FDCanUdp.ioc            CubeMX project (both cores)
├── CM4/Core/               CM4: main loop, mab_md.c (MD80 protocol), md_bench.c (benchmark / control loop), fdcan.c
├── CM7/Core/               CM7: main loop, udp_server.c, cm7_log.c, fault reporting
├── CM7/LWIP/               LwIP glue (generated + USER CODE fixes)
├── Common/                 Shared by both cores: ipc_log (SRAM4 log), wbc_protocol.h (UDP packets)
├── Drivers/, Middlewares/  ST HAL and LwIP (copied in by CubeMX)
├── pc/                     PC tools: wbc_udp_test.py (UDP rate/loss/latency test), serial_log.sh
├── NOTES.md                Development log, measurements, settings, pitfalls
└── Portenta_H7/            Earlier Portenta H7 prototype (reference only)
mab_can_protocol.md         MD80 CAN protocol reference
Project_Description.md      Original project brief
```

---

## Build, flash, run

1. **Build** in STM32CubeIDE: both projects (`FDCanUdp_CM4`, `FDCanUdp_CM7`). CM4 builds with `-O2`.
2. **Flash** with the CM7 run configuration, which has the CM4 image added on its Startup tab, so both cores are programmed. Alternatively, flash from the command line with `STM32_Programmer_CLI -c port=SWD -w <elf> -v -hardRst`.
3. **PC network:** set the wired port to `192.168.10.1/24` and check that `ip route get 192.168.10.10` goes via that port, not Wi-Fi.
4. **Serial log:** `FDCanUdp/pc/serial_log.sh <name>` (saves to `~/nucleo_<name>_MMDD_HHMM.txt`; quit with Ctrl-A k y).
5. **UDP test:** `python3 FDCanUdp/pc/wbc_udp_test.py --rate 500 --duration 60`. Add `--rt` with `sudo` for real-time priority. It never sets `ENABLE` unless `--enable` is given.

After every CubeMX regeneration, check: the SMPS define in both `.cproject` files, the LwIP heap override (`0x30045000`) and IP addresses in USER CODE, the CM7 linker section `RAM_ETH`, and CM4's `-O2`.

### LEDs

| LED | Meaning |
|---|---|
| LD2 yellow (network) | off: no Ethernet link · 1 Hz blink: link up, no commands · 10 Hz blink: valid commands arriving |
| LD1 green (motors) | blinking: all motors answering |
| LD3 red (motors) | blinking: motor missing or missed replies |

---

## Status

| Component | State |
|---|---|
| Nucleo clock / power configuration (SMPS, HSE bypass, 400 / 200 MHz, boot sync) | ✅ done |
| FDCAN1 + FDCAN2, 1 / 8 Mbps, both legs verified on hardware | ✅ done |
| MD80 protocol: ping, configure, impedance mode, enable / disable, 0x40 cyclic update | ✅ done |
| Both legs in parallel at 1 kHz with in-cycle retry (benchmark) | ✅ done, 0 failed cycles |
| Serial log CM4 → CM7 → USART3, fault reporting | ✅ done |
| Ethernet + LwIP on CM7, UDP server (placeholder states) | ✅ done, 150 000 / 150 000 packets at 500 Hz |
| CM7 ↔ CM4 command / state mailboxes | ⏳ next |
| CM4 control loop driven by PC commands (replaces the benchmark), safety rules | ⏳ planned (safe action to be decided) |
| End-to-end test PC → motors → PC, latency and soak test | ⏳ planned |
| 1 kHz PC rate (PC real-time setup, command-triggered CM4 cycle) | 💡 goal |

The detailed results and the history behind each item are in [`FDCanUdp/NOTES.md`](FDCanUdp/NOTES.md).
