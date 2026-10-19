#!/usr/bin/env python3
"""
wbc_udp_test.py - UDP link test for the Nucleo WBC bridge (Common/Inc/wbc_protocol.h)

Sends WbcCommandPacket at a fixed rate (like the whole-body controller will), waits
for the matching WbcStatePacket and reports lost packets and round-trip times.

    python3 wbc_udp_test.py                     # 500 Hz for 10 s, motors NOT enabled
    python3 wbc_udp_test.py --rate 250 --duration 60
    sudo python3 wbc_udp_test.py --rt           # SCHED_FIFO: less Linux timing jitter

The ENABLE flag is only set with --enable. No third-party packages needed.
"""
import argparse
import os
import socket
import struct
import sys
import time

# ─── Packet layout (must match Common/Inc/wbc_protocol.h) ───
WBC_NUM_MOTORS = 12
WBC_VERSION = 1
WBC_CMD_MAGIC = 0x31434257     # "WBC1"
WBC_STATE_MAGIC = 0x31545357   # "WST1"
WBC_CMD_ENABLE = 1 << 0

CMD_FMT = "<IHHII" + "5f" * WBC_NUM_MOTORS                 # header 16 B + 12 x 20 B
STATE_FMT = "<IHHIIIHH8s" + "fffHBB" * WBC_NUM_MOTORS      # header 32 B + 12 x 16 B
CMD_SIZE = struct.calcsize(CMD_FMT)
STATE_SIZE = struct.calcsize(STATE_FMT)
assert CMD_SIZE == 256, CMD_SIZE
assert STATE_SIZE == 224, STATE_SIZE

STATUS_BITS = {0: "ENABLED", 1: "CMD_TIMEOUT", 2: "MOTOR_FAULT", 3: "NO_CM4_DATA"}


def build_command(seq, enable):
    motors = [0.0] * (5 * WBC_NUM_MOTORS)   # pos, vel, tau, kp, kd = 0 for every motor
    flags = WBC_CMD_ENABLE if enable else 0
    pc_time_us = (time.monotonic_ns() // 1000) & 0xFFFFFFFF
    return struct.pack(CMD_FMT, WBC_CMD_MAGIC, WBC_VERSION, flags, seq, pc_time_us, *motors)


def parse_state(data):
    v = struct.unpack(STATE_FMT, data)
    return {"magic": v[0], "version": v[1], "status": v[2], "seq": v[3],
            "stm_us": v[4], "cm4_cycle": v[5], "cycle_us": v[6], "cmd_age_us": v[7]}


def status_text(status):
    names = [n for b, n in STATUS_BITS.items() if status & (1 << b)]
    return "|".join(names) if names else "0"


def percentile(sorted_vals, pct):
    if not sorted_vals:
        return float("nan")
    k = min(len(sorted_vals) - 1, max(0, int(round(pct / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[k]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.10.10")
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--rate", type=float, default=500.0, help="commands per second (WBC rate)")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--enable", action="store_true", help="set the ENABLE flag (motors may move!)")
    ap.add_argument("--rt", action="store_true", help="use SCHED_FIFO (needs root)")
    args = ap.parse_args()

    if args.enable:
        print("WARNING: ENABLE flag set - commands are zero, but motors will be enabled.")
    if args.rt:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
            print("SCHED_FIFO priority 50")
        except PermissionError:
            print("--rt needs root; continuing without it")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.connect((args.ip, args.port))
    sock.setblocking(False)

    # Safety: the Nucleo is on a direct cable. If the wired port is down, Linux routes the
    # address through the default gateway (Wi-Fi) and the packets go to some other machine.
    local_ip = sock.getsockname()[0]
    if local_ip.rsplit(".", 1)[0] != args.ip.rsplit(".", 1)[0]:
        print(f"ERROR: {args.ip} would be reached from {local_ip}, not from the wired port "
              f"(expected {args.ip.rsplit('.', 1)[0]}.x). Is the Ethernet cable plugged in and "
              f"the PC port up with 192.168.10.1/24?  Check: ip -br addr")
        return 2

    period_ns = int(1e9 / args.rate)
    total = int(args.rate * args.duration)
    rtt_us, send_gaps_us = [], []
    lost = late = bad = 0
    first_state = None
    last_send_ns = None

    print(f"Sending {total} commands at {args.rate:.0f} Hz to {args.ip}:{args.port} "
          f"(command {CMD_SIZE} B, state {STATE_SIZE} B)")
    next_ns = time.perf_counter_ns()
    for seq in range(1, total + 1):
        # wait for the send time: sleep coarsely, then spin the last 0.3 ms
        while True:
            remaining = next_ns - time.perf_counter_ns()
            if remaining <= 0:
                break
            if remaining > 300_000:
                time.sleep((remaining - 300_000) / 1e9)
        sent_ns = time.perf_counter_ns()
        if last_send_ns is not None:
            send_gaps_us.append((sent_ns - last_send_ns) / 1000.0)
        last_send_ns = sent_ns
        sock.send(build_command(seq, args.enable))

        # collect the reply until the next send time (90 % of the period)
        deadline = sent_ns + int(period_ns * 0.9)
        got = False
        while time.perf_counter_ns() < deadline:
            try:
                data = sock.recv(512)
            except BlockingIOError:
                continue
            now_ns = time.perf_counter_ns()
            if len(data) != STATE_SIZE:
                bad += 1
                continue
            st = parse_state(data)
            if st["magic"] != WBC_STATE_MAGIC or st["version"] != WBC_VERSION:
                bad += 1
                continue
            if st["seq"] != seq:
                late += 1          # reply to an earlier command arriving after its deadline
                continue
            if first_state is None:
                first_state = st
            rtt_us.append((now_ns - sent_ns) / 1000.0)
            got = True
            break
        if not got:
            lost += 1
        next_ns += period_ns

    sock.close()
    rtt = sorted(rtt_us)
    gaps = sorted(send_gaps_us)
    print()
    print(f"sent {total}  replies {len(rtt)}  lost {lost}  late {late}  bad {bad}")
    if rtt:
        print(f"RTT (us): avg {sum(rtt) / len(rtt):.0f}  min {rtt[0]:.0f}  p50 {percentile(rtt, 50):.0f}  "
              f"p99 {percentile(rtt, 99):.0f}  max {rtt[-1]:.0f}")
    if gaps:
        print(f"PC send period (us): target {period_ns / 1000:.0f}  p1 {percentile(gaps, 1):.0f}  "
              f"p99 {percentile(gaps, 99):.0f}  max {gaps[-1]:.0f}")
    if first_state:
        print(f"first reply: status {status_text(first_state['status'])}  seq {first_state['seq']}  "
              f"stm_us {first_state['stm_us']}")
    return 0 if lost == 0 and bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
