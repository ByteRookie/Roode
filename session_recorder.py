#!/usr/bin/env python3
"""Record calibration sessions from a Roode device.

This utility subscribes to a device's JSON message stream and appends each
payload to a ``session.json`` file.  When a message with ``type`` equal to
``scan_complete`` is received the session is closed and an optional analysis
command is executed.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

import websockets


async def record_session(
    url: str,
    session_dir: Path,
    analysis_cmd: Optional[Iterable[str]],
    scan_time_cap: Optional[float],
) -> None:
    """Subscribe to ``url`` and write JSON messages to ``session_dir``.

    Args:
        url: WebSocket URL of the device.
        session_dir: Directory where ``session.json`` is stored.
        analysis_cmd: Command to run after ``scan_complete``.
    """
    session_dir.mkdir(parents=True, exist_ok=True)
    session_file = session_dir / "session.json"

    start_time = datetime.now()
    last_scan_start: Optional[datetime] = None
    async with websockets.connect(url) as ws, session_file.open("a", encoding="utf-8") as fh:
        async for message in ws:
            try:
                payload = json.loads(message)
            except json.JSONDecodeError:
                fh.write(message.strip() + "\n")
                continue

            now = datetime.now()
            payload.setdefault("timestamp", now.isoformat())

            if payload.get("type") == "passive_scan":
                if last_scan_start:
                    payload["scan_duration"] = (now - last_scan_start).total_seconds()
                last_scan_start = now
                if scan_time_cap and payload.get("grid") == 16:
                    if (now - start_time).total_seconds() > scan_time_cap:
                        fh.write(json.dumps(payload) + "\n")
                        break

            fh.write(json.dumps(payload) + "\n")

            if payload.get("type") == "scan_complete":
                break

    if analysis_cmd:
        subprocess.run(list(analysis_cmd) + [str(session_dir)], check=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url", help="WebSocket URL of the device")
    parser.add_argument(
        "session_dir",
        nargs="?",
        default=None,
        help="Directory to store session data (default: calibration_<timestamp>)",
    )
    parser.add_argument(
        "--analysis",
        nargs=argparse.REMAINDER,
        help="Command to run after scan_complete (session path appended)",
    )
    parser.add_argument(
        "--scan-time-cap",
        type=float,
        default=None,
        help="Abort remaining 16x16 scans after given seconds",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session_dir = (
        Path(args.session_dir)
        if args.session_dir
        else Path(datetime.now().strftime("calibration_%Y-%m-%d_%H-%M"))
    )
    asyncio.run(record_session(args.url, session_dir, args.analysis, args.scan_time_cap))


if __name__ == "__main__":
    main()
