"""Copperhead switch interface."""

from mpf.platforms.interfaces.switch_platform_interface import SwitchPlatformInterface


class CopperheadSwitch(SwitchPlatformInterface):
    """A switch addressed by board number and flattened input number."""

    __slots__ = ["connection", "address"]

    def __init__(self, config, number, platform, connection, address):
        super().__init__(config, number, platform)
        self.connection = connection
        self.address = address

    def get_board_name(self):
        return "Copperhead {}".format(self.connection.board_number)
