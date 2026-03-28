from __future__ import annotations

import contextlib
import dataclasses
import os
import threading
import time
from typing import Any

from output_sensor_protocol import (
    HelloMessage,
    MSG_FAULT,
    MSG_HELLO,
    MSG_SENSOR_SAMPLE,
    MSG_STATUS,
    SensorSampleMessage,
    StatusMessage,
    FaultMessage,
    FrameDecoder,
    decode_message,
    encode_mark_home,
    encode_request_status,
    encode_set_zero,
    encode_stream_disable,
    encode_stream_enable,
)

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    serial = None


@dataclasses.dataclass(frozen=True)
class OutputSensorBridgeConfig:
    port: str
    baudrate: int = 921600
    sample_rate_hz: int = 400
    read_timeout_s: float = 0.05
    connect_timeout_s: float = 1.0
    auto_stream: bool = True
    gear_ratio: float = 25.0
    output_sign: float = -1.0


class OutputSensorBridge:
    def __init__(self, config: OutputSensorBridgeConfig) -> None:
        self._config = config
        self._serial = None
        self._lock = threading.Lock()
        self._decoder = FrameDecoder()
        self._thread = None
        self._stop = threading.Event()
        self._seq = 1
        self._hello: HelloMessage | None = None
        self._status: StatusMessage | None = None
        self._sample: SensorSampleMessage | None = None
        self._fault: FaultMessage | None = None
        self._last_error: str | None = None
        self._last_frame_time_s: float | None = None
        self._last_connect_time_s: float | None = None

    @property
    def config(self) -> OutputSensorBridgeConfig:
        return self._config

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._open_locked()
            self._stop.clear()
            self._thread = threading.Thread(target=self._reader_loop, name="output-sensor-bridge", daemon=True)
            self._thread.start()
            if self._config.auto_stream:
                self._write_locked(encode_stream_enable(self._config.sample_rate_hz, seq=self._next_seq_locked()))
            self._write_locked(encode_request_status(seq=self._next_seq_locked()))

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            if self._serial is not None:
                with contextlib.suppress(Exception):
                    self._write_locked(encode_stream_disable(seq=self._next_seq_locked()))
            thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        with self._lock:
            self._close_locked()
            self._thread = None

    def mark_home(self) -> None:
        with self._lock:
            self._ensure_open_locked()
            self._write_locked(encode_mark_home(seq=self._next_seq_locked()))
            self._write_locked(encode_request_status(seq=self._next_seq_locked()))

    def set_zero_offset_counts(self, zero_offset_counts: int) -> None:
        with self._lock:
            self._ensure_open_locked()
            self._write_locked(encode_set_zero(int(zero_offset_counts), seq=self._next_seq_locked()))
            self._write_locked(encode_request_status(seq=self._next_seq_locked()))

    def latest_snapshot(self, *, axis_motor_turns: float | None = None, gear_ratio: float | None = None) -> dict[str, Any]:
        with self._lock:
            hello = self._hello
            status = self._status
            sample = self._sample
            fault = self._fault
            last_error = self._last_error
            last_frame_time_s = self._last_frame_time_s
            serial_open = bool(self._serial is not None)
            sample_age_s = (None if last_frame_time_s is None else max(0.0, time.time() - float(last_frame_time_s)))
            effective_output_sign = (-1.0 if float(self._config.output_sign) < 0.0 else 1.0)
            output_turns = (None if sample is None else float(sample.output_turns) * effective_output_sign)
            output_vel_turns_s = (None if sample is None else float(sample.output_vel_turns_s) * effective_output_sign)
            effective_gear_ratio = float(gear_ratio if gear_ratio is not None else self._config.gear_ratio)
            compliance_lag_turns = None
            compliance_lag_output_turns = None
            if axis_motor_turns is not None and output_turns is not None and effective_gear_ratio > 0.0:
                compliance_lag_turns = float(axis_motor_turns) - (float(output_turns) * effective_gear_ratio)
                compliance_lag_output_turns = compliance_lag_turns / effective_gear_ratio
            connected = bool(serial_open and last_error is None)
            healthy = bool(
                connected
                and sample_age_s is not None
                and sample_age_s <= 0.5
                and (fault is None or int(fault.fault_code) == 0)
            )
            return {
                "configured": True,
                "connected": bool(connected),
                "healthy": bool(healthy),
                "port": str(self._config.port),
                "baudrate": int(self._config.baudrate),
                "streaming": bool(status.streaming_enabled) if status is not None else bool(self._config.auto_stream),
                "encoder_name": (
                    hello.encoder_name
                    if hello is not None
                    else ("MT6835" if sample is not None else "unknown")
                ),
                "protocol_version": 1,
                "sample_rate_hz": (
                    int(status.sample_rate_hz)
                    if status is not None
                    else (
                        int(hello.sample_rate_hz)
                        if hello is not None
                        else (int(self._config.sample_rate_hz) if sample is not None else None)
                    )
                ),
                "output_sign": effective_output_sign,
                "last_sample_age_s": sample_age_s,
                "output_turns": output_turns,
                "output_vel_turns_s": output_vel_turns_s,
                "raw_angle_counts": (None if sample is None else int(sample.raw_angle_counts)),
                "mag_status_bits": (None if sample is None else int(sample.mag_status_bits)),
                "diag_bits": (None if sample is None else int(sample.diag_bits)),
                "zero_offset_counts": (None if status is None else int(status.zero_offset_counts)),
                "homed": (None if status is None else bool(status.homed)),
                "last_fault_code": (None if status is None else int(status.last_fault_code)),
                "fault_detail": (None if fault is None else int(fault.detail)),
                "fault_timestamp_us": (None if fault is None else int(fault.timestamp_us)),
                "last_error": last_error,
                "compliance_lag_turns": compliance_lag_turns,
                "compliance_lag_output_turns": compliance_lag_output_turns,
            }

    def wait_for_data(self, timeout_s: float = 3.0) -> bool:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            with self._lock:
                if self._hello is not None or self._status is not None or self._sample is not None or self._fault is not None:
                    return True
                last_error = self._last_error
            if last_error:
                return False
            time.sleep(0.05)
        return False

    def wait_for_status(self, timeout_s: float = 3.0, *, require_homed: bool | None = None) -> bool:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            with self._lock:
                status = self._status
                last_error = self._last_error
                if status is not None:
                    if require_homed is None or bool(status.homed) == bool(require_homed):
                        return True
            if last_error:
                return False
            time.sleep(0.05)
        return False

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with self._lock:
                    self._ensure_open_locked()
                    ser = self._serial
                if ser is None:
                    time.sleep(0.1)
                    continue
                chunk = ser.read(256)
                if not chunk:
                    continue
                for frame in self._decoder.feed(chunk):
                    message = decode_message(frame)
                    with self._lock:
                        self._last_frame_time_s = time.time()
                        self._last_error = None
                        if isinstance(message, HelloMessage):
                            self._hello = message
                        elif isinstance(message, SensorSampleMessage):
                            self._sample = message
                        elif isinstance(message, StatusMessage):
                            self._status = message
                        elif isinstance(message, FaultMessage):
                            self._fault = message
            except Exception as exc:  # pragma: no cover - hardware path
                with self._lock:
                    self._last_error = str(exc)
                    self._close_locked()
                time.sleep(0.25)

    def _open_locked(self) -> None:
        if self._serial is not None:
            return
        if serial is None:
            raise RuntimeError("pyserial is required for the output sensor bridge. Install it into .venv before using the bridge.")
        self._serial = serial.Serial(
            port=self._config.port,
            baudrate=int(self._config.baudrate),
            timeout=float(self._config.read_timeout_s),
            write_timeout=float(self._config.connect_timeout_s),
        )
        self._last_connect_time_s = time.time()
        self._last_error = None
        with contextlib.suppress(Exception):
            self._serial.reset_input_buffer()
        with contextlib.suppress(Exception):
            self._serial.reset_output_buffer()

    def _close_locked(self) -> None:
        ser = self._serial
        self._serial = None
        if ser is not None:
            with contextlib.suppress(Exception):
                ser.close()

    def _ensure_open_locked(self) -> None:
        if self._serial is None:
            self._open_locked()

    def _write_locked(self, payload: bytes) -> None:
        if self._serial is None:
            raise RuntimeError("Output sensor bridge serial port is not open")
        self._serial.write(payload)
        with contextlib.suppress(Exception):
            self._serial.flush()

    def _next_seq_locked(self) -> int:
        seq = int(self._seq) & 0xFFFF
        self._seq = (int(self._seq) + 1) & 0xFFFF
        return seq


_GLOBAL_LOCK = threading.Lock()
_GLOBAL_BRIDGE: OutputSensorBridge | None = None
_GLOBAL_CONFIG_KEY: tuple[Any, ...] | None = None


def bridge_config_from_env() -> OutputSensorBridgeConfig | None:
    port = str(os.getenv("ROBOT_OUTPUT_SENSOR_PORT", "") or "").strip()
    if not port:
        return None
    baudrate = int(str(os.getenv("ROBOT_OUTPUT_SENSOR_BAUD", "921600") or "921600"))
    sample_rate_hz = int(str(os.getenv("ROBOT_OUTPUT_SENSOR_RATE_HZ", "400") or "400"))
    gear_ratio = float(str(os.getenv("ROBOT_OUTPUT_SENSOR_GEAR_RATIO", "25") or "25"))
    raw_output_sign = float(str(os.getenv("ROBOT_OUTPUT_SENSOR_SIGN", "-1") or "-1"))
    output_sign = -1.0 if raw_output_sign < 0.0 else 1.0
    return OutputSensorBridgeConfig(
        port=port,
        baudrate=baudrate,
        sample_rate_hz=sample_rate_hz,
        gear_ratio=gear_ratio,
        output_sign=output_sign,
    )


def get_output_sensor_bridge_from_env(*, start_if_needed: bool = True) -> OutputSensorBridge | None:
    global _GLOBAL_BRIDGE, _GLOBAL_CONFIG_KEY
    config = bridge_config_from_env()
    if config is None:
        return None
    config_key = (config.port, config.baudrate, config.sample_rate_hz, config.gear_ratio, config.output_sign)
    with _GLOBAL_LOCK:
        if _GLOBAL_BRIDGE is None or _GLOBAL_CONFIG_KEY != config_key:
            if _GLOBAL_BRIDGE is not None:
                with contextlib.suppress(Exception):
                    _GLOBAL_BRIDGE.stop()
            _GLOBAL_BRIDGE = OutputSensorBridge(config)
            _GLOBAL_CONFIG_KEY = config_key
        bridge = _GLOBAL_BRIDGE
    if bridge is not None and start_if_needed:
        bridge.start()
    return bridge


def get_output_sensor_snapshot_from_env(*, axis_motor_turns: float | None = None, gear_ratio: float | None = None) -> dict[str, Any] | None:
    bridge = get_output_sensor_bridge_from_env(start_if_needed=True)
    if bridge is None:
        return None
    try:
        return bridge.latest_snapshot(axis_motor_turns=axis_motor_turns, gear_ratio=gear_ratio)
    except Exception as exc:  # pragma: no cover - hardware path
        return {
            "configured": True,
            "connected": False,
            "healthy": False,
            "port": bridge.config.port,
            "baudrate": int(bridge.config.baudrate),
            "streaming": False,
            "encoder_name": "unknown",
            "protocol_version": 1,
            "output_sign": (-1.0 if float(bridge.config.output_sign) < 0.0 else 1.0),
            "last_error": str(exc),
        }
