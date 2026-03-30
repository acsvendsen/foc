#!/usr/bin/env python3
"""
RP2040 outer-loop controller — interactive test/commissioning tool.

Usage:
    python3 rp2040_outer_loop_test.py [--port /dev/cu.usbmodemXXX]

Commands (type at prompt):
    status          — read current loop and sensor status
    enable          — enable outer loop (snaps setpoint to current position)
    disable         — disable outer loop (ODrive reverts to its own control)
    gains Kp Kd Vl  — set gains, e.g.: gains 2.5 0.4 1.0
    go <turns>      — set setpoint to <turns> output-side, e.g.: go 0.1
    nudge <delta>   — relative move from current setpoint, e.g.: nudge -0.05
    watch           — stream loop telemetry for 10 s
    q / quit        — exit

Bridge protocol (device side):
    SET_SETPOINT   0x20  int32 µturns (output)
    SET_LOOP_GAINS 0x21  int32 Kp_milli, Kd_milli, vel_limit_milli
    SET_LOOP_ENABLE 0x22 uint8 0/1
    LOOP_STATUS    0x05  (device→host)
    SENSOR_SAMPLE  0x02  (device→host)
"""

import argparse
import struct
import sys
import threading
import time
from collections import deque

import serial

# ── Protocol constants ────────────────────────────────────────────────────────
SOF0, SOF1 = 0xA5, 0x5A
PROTO_VER   = 0x01

MSG_HELLO          = 0x01
MSG_SENSOR_SAMPLE  = 0x02
MSG_STATUS         = 0x03
MSG_FAULT          = 0x04
MSG_LOOP_STATUS    = 0x05

MSG_STREAM_ENABLE  = 0x10
MSG_STREAM_DISABLE = 0x11
MSG_SET_ZERO       = 0x12
MSG_REQUEST_STATUS = 0x13
MSG_MARK_HOME      = 0x14

MSG_SET_SETPOINT    = 0x20
MSG_SET_LOOP_GAINS  = 0x21
MSG_SET_LOOP_ENABLE = 0x22

CRC_POLY  = 0x1021
CRC_INIT  = 0xFFFF
MAX_PAYLOAD = 32

MSG_NAMES = {
    MSG_HELLO: "HELLO", MSG_SENSOR_SAMPLE: "SAMPLE",
    MSG_STATUS: "STATUS", MSG_FAULT: "FAULT",
    MSG_LOOP_STATUS: "LOOP_STATUS",
}

# ── CRC-16-CCITT-FALSE ────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = CRC_INIT
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

# ── Frame encode/decode ───────────────────────────────────────────────────────
def encode_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    header = struct.pack("<BBHHh", SOF0, SOF1, PROTO_VER | (msg_type << 8), seq, len(payload))
    # rebuild properly: SOF0, SOF1, ver, type, seq(2), plen(2), payload, crc(2)
    raw = bytes([SOF0, SOF1, PROTO_VER, msg_type]) + struct.pack("<HH", seq, len(payload)) + payload
    crc = crc16(raw[2:])  # CRC over everything after SOF
    return raw + struct.pack("<H", crc)

def parse_frames(buf: bytearray) -> tuple[list[dict], bytearray]:
    frames = []
    while len(buf) >= 10:
        idx = -1
        for i in range(len(buf) - 1):
            if buf[i] == SOF0 and buf[i+1] == SOF1:
                idx = i
                break
        if idx < 0:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < 10:
            break
        plen = struct.unpack_from("<H", buf, 6)[0]
        if plen > MAX_PAYLOAD:
            del buf[:2]
            continue
        frame_len = 8 + plen + 2
        if len(buf) < frame_len:
            break
        frame = bytes(buf[:frame_len])
        expected = struct.unpack_from("<H", frame, 8+plen)[0]
        actual   = crc16(frame[2:8+plen])
        if expected != actual:
            del buf[:2]
            continue
        msg_type = frame[3]
        seq      = struct.unpack_from("<H", frame, 4)[0]
        payload  = frame[8:8+plen]
        frames.append({"type": msg_type, "seq": seq, "payload": payload})
        del buf[:frame_len]
    return frames, buf

# ── Payload decoders ──────────────────────────────────────────────────────────
def decode_sensor_sample(p: bytes) -> dict:
    if len(p) < 22:
        return {}
    ts, pos_ut, vel_ut, raw, mag, diag, _ = struct.unpack_from("<IiiiHHH", p)
    return {
        "timestamp_us": ts,
        "output_turns": pos_ut / 1e6,
        "output_vel":   vel_ut / 1e6,
        "raw_counts":   raw,
        "mag_status":   mag,
        "diag":         diag,
    }

def decode_loop_status(p: bytes) -> dict:
    if len(p) < 24:
        return {}
    sp, pos, err, vcmd, tick, enabled = struct.unpack_from("<iiiiIB", p)
    return {
        "setpoint":    sp  / 1e6,
        "position":    pos / 1e6,
        "error":       err / 1e6,
        "vel_cmd":     vcmd / 1e6,
        "tick_count":  tick,
        "enabled":     bool(enabled),
    }

def decode_status(p: bytes) -> dict:
    if len(p) < 12:
        return {}
    stream, homed, fault, zero, rate, _ = struct.unpack_from("<BBHiHH", p)
    return {"streaming": bool(stream), "homed": bool(homed),
            "fault": fault, "zero_offset": zero, "rate_hz": rate}

# ── Bridge client ─────────────────────────────────────────────────────────────
class Bridge:
    def __init__(self, port: str, baud: int = 115200):
        self._ser    = serial.Serial(port, baud, timeout=0.05)
        self._buf    = bytearray()
        self._seq    = 1
        self._lock   = threading.Lock()
        self._recent: deque[dict] = deque(maxlen=200)
        self._running = True
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        while self._running:
            try:
                chunk = self._ser.read(256)
                if chunk:
                    with self._lock:
                        self._buf.extend(chunk)
                        frames, self._buf = parse_frames(self._buf)
                    for f in frames:
                        self._recent.append(f)
            except Exception:
                break

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        return s

    def send(self, msg_type: int, payload: bytes = b""):
        frame = encode_frame(msg_type, self._next_seq(), payload)
        self._ser.write(frame)

    def drain(self, timeout_s: float = 0.3) -> list[dict]:
        time.sleep(timeout_s)
        with self._lock:
            out = list(self._recent)
            self._recent.clear()
        return out

    def close(self):
        self._running = False
        self._ser.close()

    # ── High-level commands ──────────────────────────────────────────────────
    def request_status(self):
        self.send(MSG_REQUEST_STATUS)

    def set_setpoint(self, output_turns: float):
        ut = int(round(output_turns * 1_000_000))
        self.send(MSG_SET_SETPOINT, struct.pack("<i", ut))

    def set_gains(self, kp: float, kd: float, vel_limit: float):
        self.send(MSG_SET_LOOP_GAINS, struct.pack("<iii",
            int(kp * 1000), int(kd * 1000), int(vel_limit * 1000)))

    def set_loop_enable(self, enable: bool):
        self.send(MSG_SET_LOOP_ENABLE, bytes([1 if enable else 0]))

    def latest_sensor(self) -> dict:
        with self._lock:
            for f in reversed(list(self._recent)):
                if f["type"] == MSG_SENSOR_SAMPLE:
                    return decode_sensor_sample(f["payload"])
        return None

    def latest_loop_status(self) -> dict:
        with self._lock:
            for f in reversed(list(self._recent)):
                if f["type"] == MSG_LOOP_STATUS:
                    return decode_loop_status(f["payload"])
        return None

# ── Pretty printers ───────────────────────────────────────────────────────────
def print_sensor(s: dict):
    print(f"  SENSOR  pos={s['output_turns']:+.6f} t  vel={s['output_vel']:+.4f} t/s"
          f"  mag={s['mag_status']:#04x}  diag={s['diag']:#04x}")

def print_loop(l: dict):
    arrow = "●" if l["enabled"] else "○"
    print(f"  LOOP {arrow}  sp={l['setpoint']:+.6f}  pos={l['position']:+.6f}"
          f"  err={l['error']:+.6f} t  vel_cmd={l['vel_cmd']:+.4f} motor_t/s"
          f"  ticks={l['tick_count']}")

# ── CLI ───────────────────────────────────────────────────────────────────────
def autodetect_port() -> str:
    import glob
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise RuntimeError("No usbmodem port found. Is the RP2040 plugged in?")
    # pick whichever responds with bridge frames
    SOF = bytes([SOF0, SOF1])
    for p in ports:
        try:
            s = serial.Serial(p, 115200, timeout=1)
            d = s.read(64)
            s.close()
            if SOF in d:
                return p
        except Exception:
            pass
    return ports[0]

def main():
    ap = argparse.ArgumentParser(description="RP2040 outer-loop commissioning tool")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = args.port or autodetect_port()
    print(f"Connecting to {port} …")
    br = Bridge(port)
    time.sleep(0.5)

    current_setpoint = 0.0

    s = br.latest_sensor()
    if s:
        current_setpoint = s["output_turns"]
        print(f"Output shaft at {current_setpoint:+.6f} turns")
    else:
        print("(no sensor sample yet)")

    print("\nType 'help' for commands.\n")

    while True:
        try:
            line = input("loop> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd   = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            break

        elif cmd == "help":
            print(__doc__)

        elif cmd == "status":
            br.request_status()
            time.sleep(0.2)
            s = br.latest_sensor()
            l = br.latest_loop_status()
            if s: print_sensor(s)
            if l: print_loop(l)
            if not s and not l: print("  (no frames received)")

        elif cmd == "enable":
            br.set_loop_enable(True)
            time.sleep(0.1)
            l = br.latest_loop_status()
            if l:
                current_setpoint = l["setpoint"]
                print_loop(l)
            print("Outer loop ENABLED. Setpoint locked to current position.")

        elif cmd == "disable":
            br.set_loop_enable(False)
            time.sleep(0.1)
            print("Outer loop DISABLED. ODrive running on motor encoder only.")

        elif cmd == "gains":
            if len(parts) != 4:
                print("Usage: gains <Kp> <Kd> <vel_limit>")
                continue
            try:
                kp, kd, vl = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                print("Bad numbers"); continue
            br.set_gains(kp, kd, vl)
            print(f"Gains set: Kp={kp}  Kd={kd}  vel_limit={vl} motor_t/s")

        elif cmd == "go":
            if len(parts) != 2:
                print("Usage: go <output_turns>"); continue
            try:
                target = float(parts[1])
            except ValueError:
                print("Bad number"); continue
            br.set_setpoint(target)
            current_setpoint = target
            print(f"Setpoint → {target:+.6f} output turns")

        elif cmd == "nudge":
            if len(parts) != 2:
                print("Usage: nudge <delta_turns>"); continue
            try:
                delta = float(parts[1])
            except ValueError:
                print("Bad number"); continue
            current_setpoint += delta
            br.set_setpoint(current_setpoint)
            print(f"Setpoint → {current_setpoint:+.6f} output turns (Δ{delta:+.4f})")

        elif cmd == "watch":
            print("Watching for 10 s (Ctrl-C to stop early)…")
            br.drain(0)
            deadline = time.time() + 10
            try:
                while time.time() < deadline:
                    time.sleep(0.1)
                    frames = br.drain(0)
                    for f in frames:
                        t = f["type"]
                        if t == MSG_SENSOR_SAMPLE:
                            print_sensor(decode_sensor_sample(f["payload"]))
                        elif t == MSG_LOOP_STATUS:
                            print_loop(decode_loop_status(f["payload"]))
            except KeyboardInterrupt:
                pass

        elif cmd == "home":
            # Mark current position as zero
            br.send(MSG_MARK_HOME)
            print("Homed — current position is now 0.0")

        else:
            print(f"Unknown command: {cmd!r}  (type 'help')")

    br.close()
    print("Disconnected.")

if __name__ == "__main__":
    main()
