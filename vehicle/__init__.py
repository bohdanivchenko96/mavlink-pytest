import time

from .base import ConnectionStats, mavlink_enum_name
from .attitude import AttitudeComponent
from .flight import FlightComponent
from .gps import GpsComponent
from .status import StatusComponent


class Vehicle:
    def __init__(self, connection):
        self._mav = connection
        self.status = StatusComponent(connection)
        self.gps = GpsComponent(connection)
        self.attitude = AttitudeComponent(connection)
        self.flight = FlightComponent(connection)

    def receive_heartbeat(self, timeout: int = 5):
        msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError("No heartbeat received — is ArduCopter running?")
        return msg

    def vehicle_type(self) -> str:
        return mavlink_enum_name("MAV_TYPE", self._mav.mav_type)

    def connection_stats(self, duration: float = 5.0, heartbeat_timeout: int = 2) -> ConnectionStats:
        deadline = time.monotonic() + duration
        received = 0
        missed = 0
        while time.monotonic() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=heartbeat_timeout)
            if msg is not None:
                received += 1
            else:
                missed += 1
        return ConnectionStats(received=received, missed=missed)
