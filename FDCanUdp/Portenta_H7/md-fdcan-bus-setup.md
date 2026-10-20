# MAB MD FDCAN bus setup (Portenta H7 M4)


Portenta H7 M4 core talks FDCAN1 (PH13 TX / PB8 RX) to 6 MAB MD drivers, IDs {10,20,30,40,50,60}, 1 Mbps nominal / 8 Mbps data BRS. Ping (read firmwareVersion 0x802) confirmed working on hardware 2026-10-17, RTT ~100 us per motor.

Mixed firmware: id 20 = v2.5.4r, ids 10/30/40 = v2.4.0r, ids 50/60 = v2.4.1r. Firmware 2.4.x replies with header byte 0x00 instead of echoing 0x41, so never validate rx[0] — only reject 0xA1 error frames and check the echoed register address (same as CANdle-SDK).

**Why:** strict rx[0]==0x41 check falsely rejected 5 of 6 motors.
**How to apply:** any new MD read/write parsing in fdcan_udp/fdcan_m4/fdcan_m4.ino must tolerate the 0x00 header; watch for other 2.4 vs 2.5 differences when adding motion commands. M4 output goes via SerialRPC → serial_bridge_m7 sketch on the M7.

## Write frames: 0x42 vs 0x40

Firmware 2.4.x silently ignores WRITE_REGISTER (0x42): no reply at all, not even an 0xA1 error frame. Firmware 2.5.4 accepts it. Use WRITE_REGISTER_LEGACY (0x40) for all writes. Every motor answers it with its default state response (header 0xA0), which also serves as the acknowledgement.

## 0x40 update timing (motor 20 only, fw 2.5.4, 8 Mbps)

Full update (32-byte 0x40 command, then the state reply): avg 212 us, min 192 us, max 251 us. No missed replies in about 18,000 cycles at 1 kHz. The fw 2.5.4 reply is 24 bytes long; its layout is not confirmed yet.

## 6-motor timing (all enabled, zero-gain impedance, 0x40)

- Sequential (send, wait for reply, next motor): avg 1175 us, min 1126 us, max 1226 us for 6 motors, about 196 us per motor. This is too slow for 1 kHz (it ran at 854 Hz). No missed replies.
- Pipelined (queue all 6 commands, then collect): failed while automatic retransmission was off. A motor's reply (low CAN ID) wins arbitration over the queued commands for higher IDs, and the losing frames are dropped. Motor 30 always collided with motor 10's reply. Fix: turn automatic retransmission on, and cancel pending TX requests on timeout.
- All replies (fw 2.4 and 2.5) are 24 bytes with header 0xA0. Byte 3 reads 26 C on the fw 2.4 motors but 0 on motor 20 (fw 2.5).
- With automatic retransmission on, pipelined is stable: avg 841 us, min 810 us, max 890 us for 6 motors at 1 kHz, 0 missed. Sequential was avg 1188 us, which can't hold 1 kHz.
- Possible timebase issue on the M4: 2036 cycles of 1000 micros() fit in 2000 ms of millis(), about 1.8 % apart. The benchmark sketch checks both against the DWT cycle counter.

## Benchmark, free-running (PIPE, 6 motors)

| Command | Rate | avg | p99 | max | Failed cycles |
|---|---|---|---|---|---|
| PVT 20 B | 1337 Hz | 749 us | 790 us | 793 us | 13 of 4010 (motors 40 and 50, never 2 in a row) |
| TAU 8 B | 1552 Hz | 647 us | 690 us | 695 us | 0 of 4656 |

- Bus errors stayed at 0 (TEC, REC, TX failures, RX overflows).
- Replies arrive in CAN ID order. TAU latency per motor: 156, 218, 363, 430, 574, 639 us.
- TAU at about 650 us is roughly the bus limit for 6 motors: each motor costs about 45 us for its command plus about 57 us for its 24-byte reply, and the 1 Mbps arbitration phase is fixed. Going faster means fewer motors per bus, for example splitting across FDCAN1 and FDCAN2.

## 0x40 reply layout (confirmed from raw bytes)

24 bytes, little-endian:

| Bytes | Field | Notes |
|---|---|---|
| 0 | header 0xA0 | |
| 1-2 | quickStatus u16 | 0x0000 on all motors |
| 3 | temperature u8, deg C | 26 on fw 2.4; always 0 on fw 2.5.4 |
| 4-7 | position f32, rad | matches the positions read earlier |
| 8-11 | velocity f32, rad/s | |
| 12-15 | torque f32, Nm | |
| 16-19 | f32 | fw 2.5.4 only (0 on fw 2.4): 1.13169 vs main position 1.13146, probably output encoder position |
| 20-23 | f32 | fw 2.5.4 only (0 on fw 2.4): probably output encoder velocity |

CAN watchdog: 100 ms on the fw 2.4 motors, 200 ms on motor 20 (fw 2.5.4). A drive whose watchdog runs out stops, so the loop must never pause longer than 100 ms while the motors are enabled.

## Benchmark, full log (6 motors, 3 s per step; µs from micros(), which matches the DWT cycle counter)

millis() runs about 1.7 % slow on the M4, so the Hz figures below (computed with millis) read about 1.7 % high. Use micros() for timing.

| Step | avg | p99 | max | Failed cycles |
|---|---|---|---|---|
| SEQ  FULL 32 B @1 kHz | 1226 us | 1280 us | (outlier: first cycle printed RAW) | 0, but every cycle overran the period |
| PIPE FULL 32 B @1 kHz | 849 us | 890 us | 896 us | 0 / 3053 |
| PIPE PVT 20 B @1 kHz | 746 us | 790 us | 793 us | 3 / 3053 (motor 40) |
| PIPE TAU 8 B @1 kHz | 647 us | 690 us | 693 us | 0 / 3053 |
| PIPE FULL 32 B free-run | 844 us | 890 us | 893 us | 0 / 3578 |
| PIPE PVT 20 B free-run | 749 us | 790 us | 793 us | 13 / 4010 (motors 40 and 50) |
| PIPE TAU 8 B free-run | 647 us | 690 us | 695 us | 0 / 4656 |

Only the 20-byte command loses replies, both at 1 kHz and free-running, and only on motors 40 and 50 (fw 2.4.0 and 2.4.1). The longer 32-byte frame never lost a reply, so idle time and bus load are not the cause; the 20-byte frame length is the suspect. A miss means no reply at all (bus error counters stayed at 0). Avoid 20-byte commands until this is understood.
