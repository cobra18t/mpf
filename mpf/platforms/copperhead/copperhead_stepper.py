"""Copperhead SPI Motor Board stepper support."""

import asyncio
import math

from mpf.platforms.interfaces.stepper_platform_interface import StepperPlatformInterface
from mpf.platforms.copperhead.copperhead_protocol import Command, packet


class CopperheadStepper(StepperPlatformInterface):
    """A dynamically indexed Copperhead Motor Board stepper."""

    __slots__ = ["platform", "connection", "index", "home_switch", "microstep", "current_limit",
                 "acceleration", "_homing", "_enabled", "_segments", "_firmware_position",
                 "_move_start_position", "_segment_delta", "_segment_target",
                 "_segment_deadline"]

    def __init__(self, platform, connection, index, home_switch, microstep, current_limit, acceleration):
        self.platform = platform
        self.connection = connection
        self.index = index
        self.home_switch = home_switch
        self.microstep = microstep
        self.current_limit = current_limit
        self.acceleration = acceleration
        self._homing = False
        self._enabled = False
        self._segments = []
        self._firmware_position = 0
        self._move_start_position = 0
        self._segment_delta = 0
        self._segment_target = 0
        self._segment_deadline = 0
        self._send_command(False, False, False)

    def _send_command(self, enable, home, positive):
        acceleration = 0 if home else self.acceleration
        flags = ((1 if enable else 0) | (2 if home else 0) |
                 (4 if positive else 0) | (self.microstep << 3) |
                 (acceleration << 5))
        self.connection.send(packet(
            Command.STEPPER_COMMAND, self.index, flags,
            self.current_limit, self.home_switch[1]))
        self._enabled = enable

    def home(self, direction):
        """Home continuously in hardware until the configured raw switch is active."""
        self._segments = []
        self._homing = True
        self._send_command(True, True, direction == "clockwise")

    async def wait_for_move_completed(self):
        """Wait for homing or poll firmware until the absolute target is reached."""
        if self._homing:
            key = "{}-{}".format(*self.home_switch)
            while self._homing and not self.platform.switch_states.get(key, False):
                await asyncio.sleep(0.01)
            if self._homing:
                self._homing = False
                self._send_command(True, False, False)
                self._firmware_position = 0
            return

        while True:
            position = await self._read_position()
            if position == self._segment_target:
                if not self._segments:
                    return
                delta, duration_ms = self._segments.pop(0)
                self._submit_segment(delta, duration_ms)
            elif self.platform.machine.clock.get_time() >= self._segment_deadline:
                raise TimeoutError(
                    "Copperhead stepper {} did not reach position {}".format(
                        self.index, self._segment_target))
            await asyncio.sleep(0.01)

    async def _read_position(self):
        """Read one authoritative position without abandoning an in-flight response."""
        transaction = asyncio.create_task(
            self.connection.transact(Command.GET_STEPPER_POSITION, self.index))
        try:
            data = await asyncio.shield(transaction)
        except asyncio.CancelledError:
            data = await transaction
            self._firmware_position = int.from_bytes(data[1:5], "big", signed=True)
            raise
        if data[0] != self.index:
            raise ValueError("Copperhead returned position for the wrong stepper")
        self._firmware_position = int.from_bytes(data[1:5], "big", signed=True)
        return self._firmware_position

    def move_rel_pos(self, position, speed=None):
        """Move a signed relative number of steps at an optional steps/second limit."""
        position = int(position)
        if not -(1 << 31) <= position < (1 << 31):
            raise AssertionError("Copperhead stepper moves must fit in a signed 32-bit value")
        self._homing = False
        self._segments = self._build_segments(position, speed)
        self._move_start_position = self._firmware_position
        self._send_command(True, False, False)
        delta, duration_ms = self._segments.pop(0)
        self._submit_segment(delta, duration_ms)

    @staticmethod
    def _build_segments(position, speed):
        if not speed or speed <= 0 or position == 0:
            return [(position, 0)]
        speed = min(float(speed), 4000.0)
        maximum_delta = max(1, int(speed * 65.535))
        count = max(1, math.ceil(abs(position) / maximum_delta))
        quotient, remainder = divmod(abs(position), count)
        sign = 1 if position >= 0 else -1
        return [(sign * (quotient + (1 if index < remainder else 0)),
                 min(65535, math.ceil((quotient + (1 if index < remainder else 0)) * 1000 / speed)))
                for index in range(count)]

    def _submit_segment(self, delta, duration_ms):
        target = self._firmware_position + delta
        if not -(1 << 31) <= target < (1 << 31):
            raise AssertionError("Copperhead stepper targets must fit in a signed 32-bit value")
        self._segment_target = target
        minimum_seconds = (duration_ms / 1000.0 if duration_ms else
                           abs(delta) / 4000.0)
        self._segment_deadline = (
            self.platform.machine.clock.get_time() + minimum_seconds * 2.0 + 10.0)
        self.platform.send_interpolation(
            self.connection, 0x4000 + self.index * 4,
            int(target).to_bytes(4, "big", signed=True), duration_ms)
        self._segment_delta = delta

    def get_current_position_delta(self):
        """Return movement measured by the firmware since the command began."""
        return self._firmware_position - self._move_start_position

    def move_vel_mode(self, velocity):
        """Velocity mode is unnecessary with Copperhead hardware homing."""
        del velocity
        raise AssertionError("Copperhead supports hardware homing, not arbitrary velocity mode")

    def stop(self):
        """Cancel motion and disable the stepper driver."""
        self.connection.send(self.prepare_stop())

    def prepare_stop(self):
        """Cancel motion and return the driver-disable command without sending it."""
        self._segments = []
        self._homing = False
        self._move_start_position = self._firmware_position
        self._segment_delta = 0
        self._segment_target = self._firmware_position
        self._segment_deadline = 0
        self._enabled = False
        flags = (self.microstep << 3) | (self.acceleration << 5)
        return packet(Command.STEPPER_COMMAND, self.index, flags,
                      self.current_limit, self.home_switch[1])

    def set_home_position(self):
        """Hardware homing establishes position zero automatically."""
