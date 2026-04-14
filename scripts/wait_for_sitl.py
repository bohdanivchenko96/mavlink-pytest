import os
import sys
import time

from pymavlink import mavutil

MAX_WAIT_S = 60
RETRY_INTERVAL_S = 2


def wait_for_sitl(host: str) -> None:
    deadline = time.monotonic() + MAX_WAIT_S
    while time.monotonic() < deadline:
        try:
            conn = mavutil.mavlink_connection(host)
            conn.wait_heartbeat(timeout=5)
            conn.close()
            print("SITL is ready")
            return
        except Exception:
            time.sleep(RETRY_INTERVAL_S)

    print("SITL did not become ready within 60s", file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    wait_for_sitl(os.environ.get("MAVLINK_CONN", "tcp:localhost:5760"))
