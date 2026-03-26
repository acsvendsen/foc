#!/usr/bin/env python3
"""Force-flash a legacy ODrive v3.x clone when DFU OTP is blank.

Use this only when:
- the runtime device already reports a trustworthy board triplet, and
- legacy DFU refuses to flash because DFU-mode OTP does not advertise it.

This script preserves odrivetool's normal legacy DFU data path and only
overrides the missing board identity with the runtime-reported board.
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import os
from pathlib import Path

from odrive.device_manager import find_async
from odrive.firmware import FirmwareFile
from odrive.hw_version import HwVersion
from odrive.legacy_dfu import DfuDeviceDiscovery, enter_dfu_mode, write_firmware
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


async def _force_flash(
    serial_number: str,
    firmware_path: str,
    board_override: HwVersion | None,
    erase_all: bool,
):
    runtime_dev: RuntimeDevice = await find_async(
        serial_number=serial_number,
        return_type=RuntimeDevice,
    )
    runtime_board = runtime_dev.board
    if runtime_board is None:
        raise RuntimeError("Runtime board identity is unavailable; aborting forced flash.")

    selected_board = board_override or runtime_board
    if selected_board.product_line != 3 or selected_board.version != 6:
        raise RuntimeError(
            f"Selected board {selected_board.dot_string} is not a v3.6-class board; aborting."
        )

    report = {
        "timestamp": datetime.datetime.now().isoformat(),
        "serial_number": str(serial_number),
        "firmware_path": os.path.abspath(firmware_path),
        "erase_all": bool(erase_all),
        "runtime_board": _jsonable_board(runtime_board),
        "selected_board": _jsonable_board(selected_board),
        "runtime_fw_version": list(runtime_dev.fw_version or []),
    }

    async with DfuDeviceDiscovery() as dfu_discoverer:
        dfu_dev = await enter_dfu_mode(runtime_dev, dfu_discoverer)
        dfu_dev.init(ask=False)

        detected_board = getattr(dfu_dev, "board", None)
        report["dfu_board_before_override"] = _jsonable_board(detected_board)

        if detected_board == HwVersion(3, 0, 0):
            dfu_dev.board = selected_board
            report["dfu_board_override_applied"] = True
        elif detected_board is None:
            raise RuntimeError("DFU board detection returned None; aborting forced flash.")
        else:
            report["dfu_board_override_applied"] = False

        await write_firmware(
            dfu_dev,
            FirmwareFile.from_file(firmware_path),
            dfu_discoverer,
            erase_all=bool(erase_all),
            installing_bootloader=False,
        )

    reflashed_dev: RuntimeDevice = await find_async(
        serial_number=serial_number,
        return_type=RuntimeDevice,
    )
    report["post_flash_board"] = _jsonable_board(reflashed_dev.board)
    report["post_flash_fw_version"] = list(reflashed_dev.fw_version or [])
    return report


def main():
    parser = argparse.ArgumentParser(
        description="Force-flash a legacy v3.6 clone by carrying runtime board identity into DFU."
    )
    parser.add_argument(
        "--serial-number",
        required=True,
        help="USB hex serial number, e.g. 347D37763235",
    )
    parser.add_argument(
        "--firmware",
        required=True,
        help="Path to the .elf firmware image to flash.",
    )
    parser.add_argument(
        "--board",
        default=None,
        help="Optional manual board triplet override, e.g. 3.6.56. Defaults to runtime-reported board.",
    )
    parser.add_argument(
        "--no-erase-all",
        action="store_true",
        help="Preserve unrelated flash sectors. Default is full erase for normalization.",
    )
    parser.add_argument(
        "--report",
        default=None,
        help="Optional JSON path for the flash report.",
    )
    parser.add_argument(
        "--yes-i-know-this-overrides-dfu-board-check",
        action="store_true",
        help="Required safety gate.",
    )
    args = parser.parse_args()

    if not args.yes_i_know_this_overrides_dfu_board_check:
        raise SystemExit(
            "Refusing to run without --yes-i-know-this-overrides-dfu-board-check"
        )

    board_override = None if not args.board else HwVersion.from_string(args.board)
    report = asyncio.run(
        _force_flash(
            serial_number=str(args.serial_number).strip(),
            firmware_path=str(args.firmware),
            board_override=board_override,
            erase_all=not bool(args.no_erase_all),
        )
    )

    report_path = args.report
    if not report_path:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = f"logs/mks_force_legacy_flash_{stamp}.json"
    report_path = os.path.abspath(report_path)
    Path(report_path).parent.mkdir(parents=True, exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"report_saved={report_path}")


if __name__ == "__main__":
    main()
