"""Copperhead SPI Motor Board brushed-DC motor support."""

from types import SimpleNamespace

from mpf.platforms.interfaces.motor_platform_interface import MotorPlatformInterface
from mpf.platforms.copperhead.copperhead_protocol import Command, packet


class CopperheadDCMotor(MotorPlatformInterface):

    """One bidirectional motor driven by Motor Board output bits 6 and 7."""

    __slots__ = ["platform", "connection", "forward", "reverse", "_stop_handle"]

    def __init__(self, platform, connection, forward, reverse):
        """Initialize the Copperhead interface and its runtime state."""
        self.platform = platform
        self.connection = connection
        self.forward = SimpleNamespace(connection=connection, address=forward)
        self.reverse = SimpleNamespace(connection=connection, address=reverse)
        self._stop_handle = None

    def pulse(self, duration_secs, power):
        """Run forward for a bounded duration."""
        self._run(self.forward, duration_secs, power)

    def reverse_pulse(self, duration_secs, power):
        """Run in reverse for a bounded duration."""
        self._run(self.reverse, duration_secs, power)

    def _run(self, output, duration_secs, power):
        self.stop()
        power_byte = max(0, min(127, round(float(power) * 127)))
        if power_byte == 0:
            return
        # A non-auto-clearing, cancelable kick transitions to equal-power hold.
        # Preserve the Motor Board solenoid default recycle multiplier of two.
        self.connection.send(packet(
            Command.CONFIGURE_SOLENOID, output.address, 0x22,
            power_byte, 32, power_byte))
        self.platform.set_solenoid(output, True)
        self._stop_handle = self.platform.machine.clock.loop.call_later(float(duration_secs), self.stop)

    def stop(self):
        """Clear both direction outputs and cancel the pending timer."""
        self.prepare_stop()
        self.platform.set_solenoid(self.forward, False)
        self.platform.set_solenoid(self.reverse, False)

    def prepare_stop(self):
        """Cancel a pending motor stop; the caller is responsible for clearing outputs."""
        if self._stop_handle:
            self._stop_handle.cancel()
            self._stop_handle = None
