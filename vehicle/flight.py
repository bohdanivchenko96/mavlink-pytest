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

    def disarm(self, force: bool = False, timeout: int = 10) -> None:
        self._send_arm_disarm(arm=False, timeout=timeout, force=force)

    # EKF must report attitude + horizontal velocity + horizontal/vertical position
    _EKF_REQUIRED_FLAGS = 0x003B

    def wait_for_ready_to_arm(self, timeout: int = 60, required_stable_readings: int = 5) -> None:
        """Wait until GPS fix is stable and EKF has finished its GPS checks."""
        gps_stable = 0
        ekf_ready = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(
                type=["GPS_RAW_INT", "EKF_STATUS_REPORT"], blocking=True, timeout=2
            )
            if msg is None:
                continue
            if msg.get_type() == "GPS_RAW_INT":
                if msg.fix_type >= 3 and msg.satellites_visible >= 6:
                    gps_stable += 1
                else:
                    gps_stable = 0
            elif msg.get_type() == "EKF_STATUS_REPORT":
                ekf_ready = (msg.flags & self._EKF_REQUIRED_FLAGS) == self._EKF_REQUIRED_FLAGS
            if gps_stable >= required_stable_readings and ekf_ready:
                return
        raise TimeoutError(f"Vehicle did not become ready to arm within {timeout}s")

    def takeoff(self, altitude_m: float, timeout: int = 60) -> None:
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

    def _send_arm_disarm(self, arm: bool, timeout: int, force: bool = False) -> None:
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            21196 if force else 0,  # 21196 = force arm/disarm magic number
            0, 0, 0, 0, 0,
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
            elif (
                msg.get_type() == "COMMAND_ACK"
                and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            ):
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
