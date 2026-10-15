"""Copperhead lamp-matrix and NeoPixel light channels."""

from mpf.platforms.interfaces.light_platform_interface import LightPlatformInterface


class CopperheadLight(LightPlatformInterface):

    """One byte-valued Copperhead interpolation light channel."""

    __slots__ = ["platform", "connection", "address", "subtype", "output", "channel"]

    # pylint: disable-next=too-many-arguments
    def __init__(self, number, platform, connection, address, subtype, output=None, channel=None):
        """Initialize the Copperhead interface and its runtime state."""
        super().__init__(number)
        self.platform = platform
        self.connection = connection
        self.address = address
        self.subtype = subtype
        self.output = output
        self.channel = channel

    def set_fade(self, start_brightness, start_time, target_brightness, target_time):
        """Queue a hardware interpolation to the requested brightness."""
        del start_brightness, start_time
        duration_ms = max(0, min(65535, round(
            (target_time - self.platform.machine.clock.get_time()) * 1000)))
        value = max(0, min(255, round(target_brightness * 255)))
        self.platform.queue_interpolation(self.connection, self.address, value, duration_ms)

    def get_board_name(self):
        """Return the owning Copperhead board."""
        return "Copperhead {}".format(self.connection.board_number)

    def is_successor_of(self, other):
        """Return whether this immediately follows another channel."""
        return (isinstance(other, CopperheadLight) and self.connection is other.connection and
                self.subtype == other.subtype and self.address == other.address + 1 and
                (self.subtype != "neopixel" or self.output == other.output))

    def get_successor_number(self):
        """Return the next matrix or NeoPixel channel number."""
        if self.subtype == "matrix":
            return "{}-{}".format(self.connection.board_number, self.address - 0x2000 + 1)
        return "{}-{}-{}".format(self.connection.board_number, self.output, self.channel + 1)

    def __lt__(self, other):
        """Compare channels by board and output address."""
        if not isinstance(other, CopperheadLight):
            return NotImplemented
        return (self.connection.board_number, self.address) < (other.connection.board_number, other.address)
