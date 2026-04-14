from pymavlink import mavutil

from .base import BaseComponent, mavlink_enum_name


class StatusComponent(BaseComponent):

    def battery_remaining(self, timeout: int = 5) -> int:
        return self._recv("SYS_STATUS", timeout).battery_remaining

    def is_armed(self, timeout: int = 5) -> bool:
        msg = self._recv("HEARTBEAT", timeout)
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def system_status(self, timeout: int = 5) -> str:
        msg = self._recv("HEARTBEAT", timeout)
        return mavlink_enum_name("MAV_STATE", msg.system_status)
