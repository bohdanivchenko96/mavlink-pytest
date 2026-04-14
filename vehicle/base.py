from dataclasses import dataclass

from pymavlink import mavutil


def mavlink_enum_name(enum_name: str, value: int) -> str:
    return mavutil.mavlink.enums[enum_name][value].name


@dataclass
class ConnectionStats:
    received: int
    missed: int


class BaseComponent:
    def __init__(self, connection):
        self._mav = connection

    def _recv(self, msg_type: str, timeout: int = 5):
        msg = self._mav.recv_match(type=msg_type, blocking=True, timeout=timeout)
        assert msg is not None, f"{msg_type} not received — is data streaming enabled?"
        return msg
