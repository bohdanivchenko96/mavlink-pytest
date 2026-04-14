import math

from .base import BaseComponent


class AttitudeComponent(BaseComponent):

    def is_level(self, max_angle_deg: float = 10.0, timeout: int = 5) -> bool:
        msg = self._recv("ATTITUDE", timeout)
        pitch_deg = math.degrees(msg.pitch)
        roll_deg = math.degrees(msg.roll)
        return abs(pitch_deg) < max_angle_deg and abs(roll_deg) < max_angle_deg
