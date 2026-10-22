#!/usr/bin/env python3
"""
Capture the machine-readable event log from one or more LoRa nodes over USB.

Every node prints rows that start with "CSV," (see logCsv() in src/main.cpp).
This script keeps ONLY those rows, ignores the human-readable debug lines, and
stamps each row with the PC's clock. With all nodes plugged into the same PC
(Phase 2 bench / stress tests) that PC timestamp is one shared clock, so
end-to-end latency = pc_epoch_ms(DELIVER at V1) - pc_epoch_ms(NEW at origin).

On connect it also sends "time <unix-seconds>" to each node, so the node clocks
(and the hub dashboard's latency column) are set too.

Usage (Windows examples):
  python tools/capture_telemetry.py COM3
  python tools/capture_telemetry.py COM3 COM5 COM7 --out run01.csv
  python tools/capture_telemetry.py COM3 COM5 --target 300      # stop after 300 CART labels

  CART training data (passive: the node labels every heartbeat it expects from its neighbours):
  python tools/capture_telemetry.py COM8 --session F1-V1-normal --out F1-V1-normal.csv

  Final evaluation (laptop sends a test message from the node every 30 s):
  python tools/capture_telemetry.py COM5 --inject COM5 --every 30 --session E1-heap --out E1-heap.csv

  Traffic tiers (Table 4.4), one batch per run, then keeps recording until --stop-after:
  python tools/capture_telemetry.py COM8 COM5 --inject COM5 --tier medium --run 3 --stop-after 600 --out full_medium_03.csv
    low     5 messages over 5 s   (2 x k=1, 2 x k=2, 1 x k=3)
    medium 15 messages over 2 s   (5 x k=1, 6 x k=2, 4 x k=3)
    high   30 messages at once    (15 x k=1, 9 x k=2, 6 x k=3) with the node's radio silenced
           ("silent on") first; the radio comes back after --hold seconds (blackout test)
  The order of k inside a batch is shuffled with --seed + --run, so run 3 of every
  configuration gets the same sequence. Session label defaults to "<tier>-run<run>".

Outputs:
  <out>.csv        pc_epoch_ms,port,session,<node CSV columns...>
                   (data TX rows carry cart_pf, cart_class and heur_class: both relay-filter verdicts)   (load with pandas)
  <out>_raw.log    every line from every port, for debugging

Requires: pip install pyserial
"""

import argparse
import sys
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: run  pip install pyserial")

BAUD = 115200
CSV_PREFIX = "CSV,"
NODE_HEADER = ("ms,node,event,origin,id,k,type,peer,rssi,snr,peer_batt_pct,peer_traffic,"
               "peer_queue,peer_hb_age_ms,self_batt_pct,self_used_mah,mac_retries,bytes,queue_len,dtn_len,"
               "ahead,bypassed,cart_pf,cart_class,heur_class,wait_ms,queue_index,d_buffer_ms,detail")
LABEL_EVENTS = {"ACK", "NOACK", "HB_OK", "HB_MISS"}   # one labelled link attempt each (CART rows)
FAIL_EVENTS = {"NOACK", "HB_MISS"}


class Capture:
    def __init__(self, out_csv: Path, target: int, session: str):
        self.out_csv = out_csv
        self.raw_log = out_csv.with_name(out_csv.stem + "_raw.log")
        self.target = target
        self.session = session.replace(",", "_")
        self.lock = threading.Lock()
        self.rows = 0
        self.labelled = 0
        self.failed = 0
        self.header_columns = NODE_HEADER.count(",") + 1
        self.stop = threading.Event()
        self.csv_file = open(self.out_csv, "w", encoding="utf-8", newline="")
        self.raw_file = open(self.raw_log, "w", encoding="utf-8", newline="")
        self.csv_file.write("pc_epoch_ms,port,session," + NODE_HEADER + "\n")

    def handle_line(self, port: str, line: str) -> None:
        pc_ms = int(time.time() * 1000)
        with self.lock:
            self.raw_file.write(f"{pc_ms} {port} {line}\n")
            if not line.startswith(CSV_PREFIX):
                return                                   # debug line: never counted as data
            body = line[len(CSV_PREFIX):]
            if body.startswith("ms,"):
                if body != NODE_HEADER:
                    print(f"\n[{port}] WARNING: firmware CSV columns differ from this script. "
                          "Update NODE_HEADER or re-flash.", file=sys.stderr)
                return                                   # header printed at node boot
            if body.count(",") + 1 != self.header_columns:
                return                                   # incomplete row (e.g. cut by a reset)
            self.csv_file.write(f"{pc_ms},{port},{self.session},{body}\n")
            self.rows += 1
            fields = body.split(",")
            if fields[2] in LABEL_EVENTS:
                self.labelled += 1
                if fields[2] in FAIL_EVENTS:
                    self.failed += 1
            if self.rows % 10 == 0:
                self.csv_file.flush()
                self.raw_file.flush()
            share = f"{100 * self.failed / self.labelled:.0f}%" if self.labelled else "-"
            print(f"\rrows: {self.rows}   CART labels: {self.labelled}"
                  + (f"/{self.target}" if self.target else "")
                  + f"   failures (HB_MISS/NOACK): {self.failed} ({share})   ", end="", flush=True)
            if self.target and self.labelled >= self.target:
                self.stop.set()

    def close(self) -> None:
        with self.lock:
            self.csv_file.close()
            self.raw_file.close()


def open_port(name: str) -> serial.Serial:
    port = serial.Serial()
    port.port = name
    port.baudrate = BAUD
    port.timeout = 0.5
    # Keep DTR/RTS low so opening the port does not reset the ESP32.
    port.dtr = False
    port.rts = False
    port.open()
    return port


# Table 4.4: exact integer allocations (k=1, k=2, k=3) and the time the batch is spread over.
TIERS = {
    "low":    {"counts": (2, 2, 1),  "spread_s": 5.0},
    "medium": {"counts": (5, 6, 4),  "spread_s": 2.0},
    "high":   {"counts": (15, 9, 6), "spread_s": 0.0},
}
SERIAL_PACING_S = 0.05   # >= 50 ms between commands so the node's serial buffer never overflows


class TierInjector:
    """One Table 4.4 batch: a list of (send time, command) sent once, then idle."""

    def __init__(self, tier: str, run: int, seed: int, hold_s: float, start_delay_s: float):
        import random
        k1, k2, k3 = TIERS[tier]["counts"]
        ks = [1] * k1 + [2] * k2 + [3] * k3
        random.Random(seed * 1000 + run).shuffle(ks)
        n = len(ks)
        step = max(TIERS[tier]["spread_s"] / n, SERIAL_PACING_S)
        t0 = time.time() + start_delay_s
        self.schedule = []
        if tier == "high":
            self.schedule.append((t0, b"silent on\n"))
            t0 += 1.0                                  # radio off before the burst
        for i, k in enumerate(ks):
            self.schedule.append((t0 + i * step, f"sos {k} T{tier[0].upper()}{run}|{tier} run {run} msg {i + 1}/{n}\n".encode()))
        if tier == "high":
            self.schedule.append((t0 + n * step + hold_s, b"silent off\n"))
        self.tier = tier
        self.description = (f"{tier} tier, run {run}: {n} messages (k1/k2/k3 = {k1}/{k2}/{k3}) "
                            f"starting in {start_delay_s:g} s" + (f", radio off for ~{hold_s:g} s" if tier == "high" else ""))
        self.last_send_at = self.schedule[-1][0]

    def due(self) -> bool:
        return bool(self.schedule) and time.time() >= self.schedule[0][0]

    def command(self, port_name: str) -> bytes:
        return self.schedule.pop(0)[1]


class Injector:
    """Sends "sos <k> <location>|<message>" to a node on a timer (serial command in main.cpp)."""

    def __init__(self, every_s: float, priorities, session: str, offset_s: float):
        self.every_s = every_s
        self.priorities = priorities
        self.location = session[:40] or "FIELD"
        self.next_at = time.time() + 15 + offset_s   # let heartbeats/routes settle first
        self.count = 0

    def due(self) -> bool:
        return time.time() >= self.next_at

    def command(self, port_name: str) -> bytes:
        k = self.priorities[self.count % len(self.priorities)]
        self.count += 1
        self.next_at += self.every_s
        if self.next_at < time.time():                # PC slept / port stalled: do not burst
            self.next_at = time.time() + self.every_s
        return f"sos {k} {self.location}|CART test {port_name} #{self.count}\n".encode()


def reader(name: str, capture: Capture, sync_clock: bool, injector=None) -> None:
    try:
        port = open_port(name)
    except serial.SerialException as error:
        print(f"\n[{name}] could not open: {error}", file=sys.stderr)
        return
    if isinstance(injector, TierInjector):
        print(f"[{name}] connected, {injector.description}")
    else:
        print(f"[{name}] connected" + (f", sending a test message every {injector.every_s:g} s" if injector else ""))
    if sync_clock:
        port.write(f"time {time.time():.3f}\n".encode())     # Unix seconds with milliseconds
    buffer = b""
    try:
        while not capture.stop.is_set():
            if injector and injector.due():
                port.write(injector.command(name))
            # Read whatever has arrived right away (blocks at most 0.5 s only when idle),
            # so each row is stamped within a few ms of reaching the PC.
            chunk = port.read(port.in_waiting or 1)
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    capture.handle_line(name, line)
    except serial.SerialException as error:
        print(f"\n[{name}] disconnected: {error}", file=sys.stderr)
    finally:
        port.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ports", nargs="+", help="serial ports, e.g. COM3 COM5 (Windows) or /dev/ttyUSB0")
    parser.add_argument("--out", default="empirical_telemetry_dataset.csv", help="output CSV file")
    parser.add_argument("--target", type=int, default=0,
                        help="stop after this many CART labels (HB_OK/HB_MISS/ACK/NOACK rows); 0 = until Ctrl+C")
    parser.add_argument("--no-time-sync", action="store_true", help="do not send the 'time' command to nodes")
    parser.add_argument("--session", default="",
                        help="label written on every row, e.g. A-V6-200m (default: output file name). "
                             "Used to split training/test data by session.")
    parser.add_argument("--inject", nargs="+", default=[], metavar="PORT",
                        help="ports whose node should send automatic test messages (never the hub V1)")
    parser.add_argument("--every", type=float, default=30.0,
                        help="seconds between injected messages per port (default 30; keep >= 20 at SF12)")
    parser.add_argument("--priorities", default="1,2,3",
                        help="priorities to cycle through for injected messages (default 1,2,3)")
    parser.add_argument("--tier", choices=sorted(TIERS), default=None,
                        help="send one Table 4.4 batch on each --inject port instead of the --every timer")
    parser.add_argument("--run", type=int, default=1, help="run (trial) number, 1-10; sets the k order")
    parser.add_argument("--seed", type=int, default=2026, help="base seed for the k order (same for all configs)")
    parser.add_argument("--hold", type=float, default=60.0, help="high tier: seconds the radio stays off")
    parser.add_argument("--start-delay", type=float, default=120.0,
                        help="tier: seconds to wait after connecting before the batch (routes settle; default 120)")
    parser.add_argument("--stop-after", type=float, default=0.0,
                        help="tier: stop recording this many seconds after the last command (0 = Ctrl+C)")
    args = parser.parse_args()

    unknown = [p for p in args.inject if p not in args.ports]
    if unknown:
        sys.exit(f"--inject ports must also be listed as capture ports: {' '.join(unknown)}")
    if args.inject and args.every < 10:
        sys.exit("--every below 10 s would flood the SF12 channel")
    try:
        priorities = [int(k) for k in args.priorities.split(",") if k.strip()]
    except ValueError:
        sys.exit("--priorities must look like 1,2,3")
    if not priorities or any(k not in (1, 2, 3) for k in priorities):
        sys.exit("--priorities must only contain 1, 2 or 3")

    if args.tier and not args.inject:
        sys.exit("--tier needs --inject PORT (the node(s) that send the batch)")
    session = args.session or (f"{args.tier}-run{args.run:02d}" if args.tier else Path(args.out).stem)
    capture = Capture(Path(args.out), args.target, session)
    stagger = args.every / max(len(args.inject), 1)

    def make_injector(port):
        if port not in args.inject:
            return None
        if args.tier:
            return TierInjector(args.tier, args.run, args.seed, args.hold, args.start_delay)
        return Injector(args.every, priorities, session, stagger * args.inject.index(port))

    injectors = {p: make_injector(p) for p in args.ports}
    threads = [threading.Thread(target=reader, args=(p, capture, not args.no_time_sync, injectors[p]), daemon=True)
               for p in args.ports]
    stop_at = None
    if args.tier and args.stop_after > 0:
        stop_at = max(i.last_send_at for i in injectors.values() if i) + args.stop_after
    for thread in threads:
        thread.start()
    print(f"Writing {capture.out_csv} (raw log: {capture.raw_log}). Press Ctrl+C to stop.")
    try:
        while not capture.stop.is_set() and any(t.is_alive() for t in threads):
            if stop_at and time.time() >= stop_at:
                print(f"\n--stop-after reached ({args.stop_after:g} s after the last command).")
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    capture.stop.set()
    for thread in threads:
        thread.join(timeout=2)
    capture.close()
    print(f"\nSaved {capture.rows} rows ({capture.labelled} CART labels, {capture.failed} failures) "
          f"to {capture.out_csv}")


if __name__ == "__main__":
    main()
