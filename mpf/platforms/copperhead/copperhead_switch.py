"""Copperhead switch interface."""

from mpf.platforms.interfaces.switch_platform_interface import SwitchPlatformInterface


class CopperheadSwitch(SwitchPlatformInterface):

    """A switch addressed by board number and flattened input number."""

    __slots__ = ["connection", "address"]

    # pylint: disable-next=too-many-arguments
    def __init__(self, config, number, platform, connection, address):
        """Initialize the Copperhead interface and its runtime state."""
        super().__init__(config, number, platform)
        self.connection = connection
        self.address = address

    def get_board_name(self):
        """Return the owning Copperhead board name."""
        return "Copperhead {}".format(self.connection.board_number)
