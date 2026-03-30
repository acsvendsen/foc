from __future__ import annotations

import dataclasses
import struct
from typing import Any

SOF0 = 0xA5
SOF1 = 0x5A
PROTOCOL_VERSION = 0x01

MSG_HELLO = 0x01
MSG_SENSOR_SAMPLE = 0x02
MSG_STATUS = 0x03
MSG_FAULT = 0x04
MSG_LOOP_STATUS    = 0x05   # outer-loop telemetry, device→host

MSG_STREAM_ENABLE  = 0x10
MSG_STREAM_DISABLE = 0x11
MSG_SET_ZERO       = 0x12
MSG_REQUEST_STATUS = 0x13
MSG_MARK_HOME      = 0x14

MSG_SET_SETPOINT    = 0x20  # outer-loop position target (output µturns)
MSG_SET_LOOP_GAINS  = 0x21  # Kp, Kd, vel_limit (milligain i32 each)
MSG_SET_LOOP_ENABLE = 0x22  # uint8 0/1

ENCODER_KIND_UNKNOWN = 0x00
ENCODER_KIND_MT6835 = 0x01

_FRAME_HEADER  = struct.Struct("<BBBBHH")
_HELLO         = struct.Struct("<BBHI")
_SENSOR_SAMPLE = struct.Struct("<IiiIHHH")
_STATUS        = struct.Struct("<BBHiHH")
_FAULT         = struct.Struct("<HHI")
_LOOP_STATUS   = struct.Struct("<iiiiIBxxx")   # setpoint, pos, err, vel_cmd, tick, enabled + 3 pad
_STREAM_ENABLE = struct.Struct("<HH")
_SET_ZERO      = struct.Struct("<i")
_SET_SETPOINT  = struct.Struct("<i")
_SET_LOOP_GAINS = struct.Struct("<iii")        # kp_milli, kd_milli, vel_limit_milli


@dataclasses.dataclass(frozen=True)
class ProtocolFrame:
    version: int
    msg_type: int
    seq: int
    payload: bytes


@dataclasses.dataclass(frozen=True)
class HelloMessage:
    encoder_kind: int
    flags: int
    sample_rate_hz: int
    firmware_build: int

    @property
    def encoder_name(self) -> str:
        return {
            ENCODER_KIND_MT6835: "MT6835",
        }.get(int(self.encoder_kind), f"unknown({int(self.encoder_kind)})")


@dataclasses.dataclass(frozen=True)
class SensorSampleMessage:
    timestamp_us: int
    output_turns_uturn: int
    output_vel_uturn_s: int
    raw_angle_counts: int
    mag_status_bits: int
    diag_bits: int
    reserved: int

    @property
    def output_turns(self) -> float:
        return float(self.output_turns_uturn) / 1_000_000.0

    @property
    def output_vel_turns_s(self) -> float:
        return float(self.output_vel_uturn_s) / 1_000_000.0


@dataclasses.dataclass(frozen=True)
class StatusMessage:
    streaming_enabled: bool
    homed: bool
    last_fault_code: int
    zero_offset_counts: int
    sample_rate_hz: int
    reserved: int


@dataclasses.dataclass(frozen=True)
class FaultMessage:
    fault_code: int
    detail: int
    timestamp_us: int


@dataclasses.dataclass(frozen=True)
class LoopStatusMessage:
    setpoint_uturn: int    # target output position [µturns]
    position_uturn: int    # measured output position [µturns]
    error_uturn: int       # setpoint - position [µturns]
    vel_cmd_umturn_s: int  # motor velocity command [µ-motor-turns/s]
    tick_count: int
    enabled: bool

    @property
    def setpoint(self) -> float:
        return float(self.setpoint_uturn) / 1_000_000.0

    @property
    def position(self) -> float:
        return float(self.position_uturn) / 1_000_000.0

    @property
    def error(self) -> float:
        return float(self.error_uturn) / 1_000_000.0

    @property
    def vel_cmd(self) -> float:
        return float(self.vel_cmd_umturn_s) / 1_000_000.0


@dataclasses.dataclass(frozen=True)
class UnknownMessage:
    msg_type: int
    payload: bytes


DecodedMessage = HelloMessage | SensorSampleMessage | StatusMessage | FaultMessage | LoopStatusMessage | UnknownMessage


class FrameDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[ProtocolFrame]:
        if data:
            self._buffer.extend(data)
        frames: list[ProtocolFrame] = []
        minimum_size = _FRAME_HEADER.size + 2
        while len(self._buffer) >= minimum_size:
            sof_index = self._find_sof()
            if sof_index < 0:
                self._buffer.clear()
                break
            if sof_index > 0:
                del self._buffer[:sof_index]
            if len(self._buffer) < minimum_size:
                break
            sof0, sof1, version, msg_type, seq, payload_len = _FRAME_HEADER.unpack_from(self._buffer, 0)
            if sof0 != SOF0 or sof1 != SOF1:
                del self._buffer[:1]
                continue
            frame_len = _FRAME_HEADER.size + int(payload_len) + 2
            if len(self._buffer) < frame_len:
                break
            payload = bytes(self._buffer[_FRAME_HEADER.size:frame_len - 2])
            expected_crc = int.from_bytes(self._buffer[frame_len - 2:frame_len], "little")
            actual_crc = crc16_ccitt_false(bytes(self._buffer[2:frame_len - 2]))
            if actual_crc != expected_crc:
                del self._buffer[:2]
                continue
            frames.append(ProtocolFrame(version=int(version), msg_type=int(msg_type), seq=int(seq), payload=payload))
            del self._buffer[:frame_len]
        return frames

    def _find_sof(self) -> int:
        needle = bytes((SOF0, SOF1))
        return self._buffer.find(needle)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def encode_frame(msg_type: int, payload: bytes = b"", *, seq: int = 0, version: int = PROTOCOL_VERSION) -> bytes:
    header = _FRAME_HEADER.pack(SOF0, SOF1, int(version) & 0xFF, int(msg_type) & 0xFF, int(seq) & 0xFFFF, len(payload) & 0xFFFF)
    crc = crc16_ccitt_false(header[2:] + payload)
    return header + payload + int(crc).to_bytes(2, "little")


def encode_stream_enable(rate_hz: int, *, flags: int = 0, seq: int = 0) -> bytes:
    return encode_frame(MSG_STREAM_ENABLE, _STREAM_ENABLE.pack(int(rate_hz) & 0xFFFF, int(flags) & 0xFFFF), seq=seq)


def encode_stream_disable(*, seq: int = 0) -> bytes:
    return encode_frame(MSG_STREAM_DISABLE, b"", seq=seq)


def encode_request_status(*, seq: int = 0) -> bytes:
    return encode_frame(MSG_REQUEST_STATUS, b"", seq=seq)


def encode_mark_home(*, seq: int = 0) -> bytes:
    return encode_frame(MSG_MARK_HOME, b"", seq=seq)


def encode_set_zero(zero_offset_counts: int, *, seq: int = 0) -> bytes:
    return encode_frame(MSG_SET_ZERO, _SET_ZERO.pack(int(zero_offset_counts)), seq=seq)


def encode_set_setpoint(output_turns: float, *, seq: int = 0) -> bytes:
    uturn = int(round(float(output_turns) * 1_000_000.0))
    return encode_frame(MSG_SET_SETPOINT, _SET_SETPOINT.pack(uturn), seq=seq)


def encode_set_loop_gains(kp: float, kd: float, vel_limit: float, *, seq: int = 0) -> bytes:
    return encode_frame(MSG_SET_LOOP_GAINS, _SET_LOOP_GAINS.pack(
        int(round(float(kp) * 1000.0)),
        int(round(float(kd) * 1000.0)),
        int(round(float(vel_limit) * 1000.0)),
    ), seq=seq)


def encode_set_loop_enable(enable: bool, *, seq: int = 0) -> bytes:
    return encode_frame(MSG_SET_LOOP_ENABLE, bytes([1 if enable else 0]), seq=seq)


def decode_message(frame: ProtocolFrame) -> DecodedMessage:
    payload = bytes(frame.payload)
    if frame.msg_type == MSG_HELLO:
        values = _unpack_exact(_HELLO, payload, "HELLO")
        return HelloMessage(*values)
    if frame.msg_type == MSG_SENSOR_SAMPLE:
        values = _unpack_exact(_SENSOR_SAMPLE, payload, "SENSOR_SAMPLE")
        return SensorSampleMessage(*values)
    if frame.msg_type == MSG_STATUS:
        values = _unpack_exact(_STATUS, payload, "STATUS")
        streaming_enabled, homed, last_fault_code, zero_offset_counts, sample_rate_hz, reserved = values
        return StatusMessage(bool(streaming_enabled), bool(homed), last_fault_code, zero_offset_counts, sample_rate_hz, reserved)
    if frame.msg_type == MSG_FAULT:
        values = _unpack_exact(_FAULT, payload, "FAULT")
        return FaultMessage(*values)
    if frame.msg_type == MSG_LOOP_STATUS:
        values = _unpack_exact(_LOOP_STATUS, payload, "LOOP_STATUS")
        sp, pos, err, vcmd, tick, enabled = values
        return LoopStatusMessage(sp, pos, err, vcmd, tick, bool(enabled))
    return UnknownMessage(msg_type=int(frame.msg_type), payload=payload)


def message_to_dict(message: DecodedMessage) -> dict[str, Any]:
    if isinstance(message, HelloMessage):
        return {
            "kind": "hello",
            "encoder_kind": int(message.encoder_kind),
            "encoder_name": message.encoder_name,
            "flags": int(message.flags),
            "sample_rate_hz": int(message.sample_rate_hz),
            "firmware_build": int(message.firmware_build),
        }
    if isinstance(message, SensorSampleMessage):
        return {
            "kind": "sensor_sample",
            "timestamp_us": int(message.timestamp_us),
            "output_turns": float(message.output_turns),
            "output_vel_turns_s": float(message.output_vel_turns_s),
            "output_turns_uturn": int(message.output_turns_uturn),
            "output_vel_uturn_s": int(message.output_vel_uturn_s),
            "raw_angle_counts": int(message.raw_angle_counts),
            "mag_status_bits": int(message.mag_status_bits),
            "diag_bits": int(message.diag_bits),
        }
    if isinstance(message, StatusMessage):
        return {
            "kind": "status",
            "streaming_enabled": bool(message.streaming_enabled),
            "homed": bool(message.homed),
            "last_fault_code": int(message.last_fault_code),
            "zero_offset_counts": int(message.zero_offset_counts),
            "sample_rate_hz": int(message.sample_rate_hz),
        }
    if isinstance(message, FaultMessage):
        return {
            "kind": "fault",
            "fault_code": int(message.fault_code),
            "detail": int(message.detail),
            "timestamp_us": int(message.timestamp_us),
        }
    return {
        "kind": "unknown",
        "msg_type": int(message.msg_type),
        "payload_hex": message.payload.hex(),
    }


def _unpack_exact(fmt: struct.Struct, payload: bytes, name: str) -> tuple[Any, ...]:
    if len(payload) != fmt.size:
        raise ValueError(f"{name} payload size mismatch: expected {fmt.size}, got {len(payload)}")
    return fmt.unpack(payload)
