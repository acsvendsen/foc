#!/usr/bin/env python3
"""Probe legacy USB DFU write viability on an ODrive v3.x clone.

This script does not try to be a friendly end-user flasher. Its purpose is to
answer one hard question with evidence:

Can this board erase, write, and verify flash sectors over legacy USB DFU on
this machine?

It connects to the runtime device, carries the runtime board identity into DFU
when OTP is blank, writes the selected firmware sectors, reads them back
immediately in DFU, and saves a JSON report.
"""

from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import json
from pathlib import Path

from odrive.device_manager import find_async
from odrive.firmware import FirmwareFile
from odrive.hw_version import HwVersion
from odrive.legacy_dfu import (
    DfuDeviceDiscovery,
    _get_first_mismatch_index,
    _populate_sectors,
    enter_dfu_mode,
)
from odrive.runtime_device import RuntimeDevice


def _jsonable_board(board: HwVersion | None):
    if board is None:
        return None
    return {
        "product_line": int(board.product_line),
        "version": int(board.version),
        "variant": int(board.variant),
        "dot_string": board.dot_string,
        "display_name": board.display_name,
    }


def _snippet(buf: bytes | bytearray, n: int = 16):
    return " ".join(f"{b:02X}" for b in bytes(buf[:n]))


async def _run_probe(
    serial_number: str,
    firmware_path: str,
    board_override: HwVersion | None,
    start_sector: int,
    sector_count: int | None,
    jump_to_app: bool,
):
    runtime_dev: RuntimeDevice = await find_async(
        serial_number=serial_number,
        return_type=RuntimeDevice,
    )
    runtime_board = runtime_dev.board
    if runtime_board is None:
        raise RuntimeError("Runtime board identity unavailable; aborting.")

    selected_board = board_override or runtime_board
    fw = FirmwareFile.from_file(firmware_path)
    report = {
        "timestamp": dt.datetime.now().isoformat(),
        "serial_number": str(serial_number),
        "firmware_path": str(Path(firmware_path).resolve()),
        "runtime_board": _jsonable_board(runtime_board),
        "selected_board": _jsonable_board(selected_board),
        "runtime_fw_version": list(runtime_dev.fw_version or []),
        "start_sector": int(start_sector),
        "sector_count": None if sector_count is None else int(sector_count),
        "jump_to_app": bool(jump_to_app),
        "sectors": [],
    }

    async with DfuDeviceDiscovery() as dfu_discoverer:
        dfu_dev = await enter_dfu_mode(runtime_dev, dfu_discoverer)
        dfu_dev.init(ask=False)
        report["dfu_board_before_override"] = _jsonable_board(getattr(dfu_dev, "board", None))

        if getattr(dfu_dev, "board", None) == HwVersion(3, 0, 0):
            dfu_dev.board = selected_board
            report["dfu_board_override_applied"] = True
        else:
            report["dfu_board_override_applied"] = False

        sections = list(fw.get_flash_sections())
        touched = list(_populate_sectors(dfu_dev.memories["Internal Flash"]["sectors"], sections))
        if start_sector < 0 or start_sector >= len(touched):
            raise RuntimeError(f"start_sector {start_sector} out of range 0..{len(touched)-1}")
        end_sector = len(touched) if sector_count is None else min(len(touched), start_sector + sector_count)
        chosen = touched[start_sector:end_sector]
        report["touched_sector_count"] = len(touched)
        report["chosen_sector_count"] = len(chosen)

        dfu_dev.select_memory("Internal Flash")
        dfu_dev.clear_status()

        for abs_idx, (sector, expected_data) in enumerate(chosen, start=start_sector):
            step = {
                "index": int(abs_idx),
                "addr": int(sector["addr"]),
                "len": int(sector["len"]),
                "expected_head": _snippet(expected_data),
            }
            try:
                before = bytes(dfu_dev.read_sector(sector))
                step["before_head"] = _snippet(before)

                print(f"[sector {abs_idx}] erase 0x{sector['addr']:08X} len={sector['len']}")
                dfu_dev.erase_sector(sector)

                erased = bytes(dfu_dev.read_sector(sector))
                step["after_erase_head"] = _snippet(erased)
                step["erase_ff_prefix_len"] = next((i for i, b in enumerate(erased) if b != 0xFF), len(erased))

                print(f"[sector {abs_idx}] write 0x{sector['addr']:08X}")
                dfu_dev.write_sector(sector, expected_data)

                observed = bytes(dfu_dev.read_sector(sector))
                step["after_write_head"] = _snippet(observed)
                mismatch = _get_first_mismatch_index(observed, expected_data)
                step["mismatch_index"] = mismatch
                step["verified"] = mismatch is None
                if mismatch is not None:
                    mismatch -= mismatch % 16
                    step["observed_snippet"] = _snippet(observed[mismatch:mismatch + 16], 16)
                    step["expected_snippet"] = _snippet(expected_data[mismatch:mismatch + 16], 16)
                    report["sectors"].append(step)
                    report["ok"] = False
                    report["failed_sector"] = int(abs_idx)
                    report["failure_reason"] = f"verify_mismatch@0x{sector['addr'] + mismatch:08X}"
                    break

                report["sectors"].append(step)
            except Exception as exc:
                step["verified"] = False
                step["exception"] = repr(exc)
                report["sectors"].append(step)
                report["ok"] = False
                report["failed_sector"] = int(abs_idx)
                report["failure_reason"] = repr(exc)
                break
        else:
            report["ok"] = True

        if jump_to_app:
            print("Jumping back to application...")
            dfu_dev.jump_to_application(0x08000000)

    return report


def main():
    parser = argparse.ArgumentParser(description="Probe legacy DFU write/readback on an MKS/ODrive v3.x clone.")
    parser.add_argument("--serial-number", required=True, help="USB hex serial, e.g. 347D37763235")
    parser.add_argument("--firmware", required=True, help="Path to firmware .elf")
    parser.add_argument("--board", default=None, help="Optional manual board override, e.g. 3.6.56")
    parser.add_argument("--start-sector", type=int, default=0, help="First touched firmware sector to test")
    parser.add_argument("--sector-count", type=int, default=None, help="How many touched sectors to test")
    parser.add_argument("--jump-to-app", action="store_true", help="Jump back to application after probe")
    parser.add_argument("--report", default=None, help="Optional JSON report path")
    parser.add_argument(
        "--yes-i-know-this-writes-flash",
        action="store_true",
        help="Required safety gate",
    )
    args = parser.parse_args()

    if not args.yes_i_know_this_writes_flash:
        raise SystemExit("Refusing to run without --yes-i-know-this-writes-flash")

    board_override = None if not args.board else HwVersion.from_string(args.board)
    report = asyncio.run(
        _run_probe(
            serial_number=str(args.serial_number).strip(),
            firmware_path=str(args.firmware),
            board_override=board_override,
            start_sector=int(args.start_sector),
            sector_count=args.sector_count,
            jump_to_app=bool(args.jump_to_app),
        )
    )

    report_path = args.report or f"logs/mks_probe_legacy_dfu_write_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    report_path = Path(report_path).resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"report_saved={report_path}")


if __name__ == "__main__":
    main()
