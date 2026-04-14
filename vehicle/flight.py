import time

from pymavlink import mavutil

from .base import BaseComponent, mavlink_enum_name


class FlightComponent(BaseComponent):

    def get_mode(self) -> str:
        return self._mav.flightmode

    def set_mode(self, mode_name: str) -> None:
        mode_id = self._mav.mode_mapping()[mode_name]
        self._mav.mav.set_mode_send(
            self._mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def arm(self, timeout: int = 10) -> None:
        self._send_arm_disarm(arm=True, timeout=timeout)

    def disarm(self, timeout: int = 10) -> None:
        self._send_arm_disarm(arm=False, timeout=timeout)

    def wait_for_ready_to_arm(self, timeout: int = 30) -> None:
        """Wait until all pre-arm checks pass (GPS fix, EKF ready, etc.)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
            if msg and msg.fix_type >= 3 and msg.satellites_visible >= 6:
                return
        raise TimeoutError(f"Vehicle did not become ready to arm within {timeout}s")

    def takeoff(self, altitude_m: float, timeout: int = 30) -> None:
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude_m,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            if msg and msg.relative_alt / 1000.0 >= altitude_m * 0.95:
                return
        raise TimeoutError(f"Vehicle did not reach {altitude_m}m within {timeout}s")

    def land(self, timeout: int = 30) -> None:
        self.set_mode("LAND")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg and not bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
        raise TimeoutError(f"Vehicle did not land and disarm within {timeout}s")

    def _send_arm_disarm(self, arm: bool, timeout: int) -> None:
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0, 0, 0, 0, 0, 0,
        )

        # Collect all messages until COMMAND_ACK arrives, capturing any
        # STATUSTEXT messages ArduCopter sends to explain a rejection.
        status_texts = []
        ack = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                status_texts.append(msg.text.strip())
            elif msg.get_type() == "COMMAND_ACK":
                ack = msg
                break

        if ack is None:
            raise TimeoutError(f"No COMMAND_ACK received within {timeout}s")
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_name = mavlink_enum_name("MAV_RESULT", ack.result)
            detail = "; ".join(status_texts) if status_texts else "no detail available"
            raise RuntimeError(
                f"{'Arm' if arm else 'Disarm'} rejected: {result_name} — {detail}"
            )

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg:
                is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if is_armed == arm:
                    return
        raise TimeoutError(f"Vehicle did not {'arm' if arm else 'disarm'} within {timeout}s")
