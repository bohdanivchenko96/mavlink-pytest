from dataclasses import dataclass

from .base import BaseComponent


@dataclass
class Position:
    lat: float
    lon: float
    altitude_m: float


class GpsComponent(BaseComponent):

    def satellites_visible(self, timeout: int = 5) -> int:
        return self._recv("GPS_RAW_INT", timeout).satellites_visible

    def altitude(self, timeout: int = 5) -> float:
        return self._recv("GLOBAL_POSITION_INT", timeout).relative_alt / 1000.0

    def position(self, timeout: int = 5) -> Position:
        msg = self._recv("GLOBAL_POSITION_INT", timeout)
        return Position(
            lat=msg.lat / 1e7,
            lon=msg.lon / 1e7,
            altitude_m=msg.relative_alt / 1000.0,
        )
