"""Copperhead interpolated RC-servo support."""

from mpf.platforms.interfaces.servo_platform_interface import ServoPlatformInterface


class CopperheadServo(ServoPlatformInterface):

    """An RC servo addressed by its physical GPIO."""

    __slots__ = ["platform", "connection", "gpio", "speed_limit", "current_position"]

    def __init__(self, platform, connection, gpio):
        """Initialize the Copperhead interface and its runtime state."""
        self.platform = platform
        self.connection = connection
        self.gpio = gpio
        self.speed_limit = 0
        self.current_position = 0

    def go_to_position(self, position):
        """Move to a normalized position using the configured speed limit."""
        target = max(0, min(255, int(position * 255)))
        if target == 0 or self.current_position == 0 or self.speed_limit <= 0:
            duration_ms = 0
        else:
            duration_ms = min(65535, round(
                1000 * abs(target - self.current_position) / (255 * self.speed_limit)))
        self.platform.send_interpolation(self.connection, 0x3000 + self.gpio, bytes((target,)), duration_ms)
        self.current_position = target

    def set_speed_limit(self, speed_limit):
        """Set MPF servo speed; zero or less selects immediate movement."""
        self.speed_limit = speed_limit

    def set_acceleration_limit(self, acceleration_limit):
        """Copperhead does not currently implement servo acceleration limits."""
        del acceleration_limit

    def stop(self):
        """Disable servo PWM immediately using the firmware's zero position."""
        address, data, duration_ms = self.prepare_stop()
        self.platform.send_interpolation(self.connection, address, data, duration_ms)

    def prepare_stop(self):
        """Return an immediate servo-disable interpolation without sending it."""
        self.current_position = 0
        return 0x3000 + self.gpio, b"\x00", 0
