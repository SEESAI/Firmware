#!/usr/bin/env python3
"""Test 129: receive and decode SEES_GPS_RAW from PX4 SITL.

Generates the pymavlink `sees` dialect from the sees.xml that this Firmware checkout pins
(src/modules/mavlink/mavlink), with that submodule's own pymavlink, then listens on the
SITL Onboard-mode MAVLink instance (UDP 14540) and reports what arrives.

Usage, with PX4 SITL running (see test_129.md):
    python3 docs/sees/tests/scripts/129_sees_gps_raw.py [--duration 10] [--port 14540]
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
MAVLINK = REPO / "src" / "modules" / "mavlink" / "mavlink"
SEES_XML = MAVLINK / "message_definitions" / "v1.0" / "sees.xml"

EXPECTED_ID = 13000
EXPECTED_LEN = 81
EXPECTED_CRC_EXTRA = 9


def generate_dialect(out_dir: Path) -> None:
    env = dict(os.environ, PYTHONPATH=str(MAVLINK))
    subprocess.run(
        [sys.executable, "-m", "pymavlink.tools.mavgen", "--lang=Python", "--wire-protocol=2.0",
         "--output", str(out_dir / "sees.py"), str(SEES_XML)],
        cwd=MAVLINK, env=env, check=True, stdout=subprocess.DEVNULL)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--duration", type=float, default=10.0, help="seconds to listen (default 10)")
    parser.add_argument("--port", type=int, default=14540, help="UDP port to listen on (default 14540)")
    args = parser.parse_args()

    os.environ["MAVLINK20"] = "1"
    out_dir = Path(tempfile.mkdtemp(prefix="sees_dialect_"))
    generate_dialect(out_dir)
    sys.path.insert(0, str(MAVLINK))
    sys.path.insert(0, str(out_dir))
    import sees  # noqa: E402  (generated above)
    from pymavlink import mavutil  # noqa: E402

    msg_class = sees.MAVLink_sees_gps_raw_message
    crc_extra = msg_class.crc_extra
    length = msg_class.unpacker.size  # payload length, from the generated struct format
    print(f"Definition: id {msg_class.id}, CRC_EXTRA {crc_extra}, payload {length} bytes")

    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{args.port}")
    conn.mav = sees.MAVLink(conn, srcSystem=255)

    counts: dict[int, int] = {}
    first = None
    last = None
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        msg = conn.recv_match(type="SEES_GPS_RAW", blocking=True, timeout=1)
        if msg is None:
            continue
        counts[msg.gps_id] = counts.get(msg.gps_id, 0) + 1
        first = first or msg
        last = msg

    total = sum(counts.values())
    print(f"Received {total} SEES_GPS_RAW in {args.duration:.0f} s "
          f"({total / args.duration:.1f} Hz), per gps_id: {dict(sorted(counts.items()))}")
    if last is not None:
        print("Last message:")
        for key, value in last.to_dict().items():
            if key != "mavpackettype":
                print(f"  {key}: {value}")

    checks = [
        ("message id is 13000", msg_class.id == EXPECTED_ID),
        ("CRC_EXTRA is 9", crc_extra == EXPECTED_CRC_EXTRA),
        ("payload is 81 bytes", length == EXPECTED_LEN),
        ("messages received", total > 0),
    ]
    if last is not None:
        checks += [
            ("lat/lon set", last.lat != 0 and last.lon != 0),
            ("heading_accuracy is NaN or finite >= 0",
             math.isnan(last.heading_accuracy) or last.heading_accuracy >= 0),
        ]
    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
