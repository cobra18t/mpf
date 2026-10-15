"""Copperhead solenoid interface."""

from mpf.platforms.interfaces.driver_platform_interface import DriverPlatformInterface


class CopperheadDriver(DriverPlatformInterface):
    """A solenoid addressed by board number and flattened output number."""

    __slots__ = ["platform", "connection", "address", "platform_settings", "switch_rule",
                 "switch_addresses", "eos_switch_address", "iflip_analog_input",
                 "iflip_eos_start_count"]

    def __init__(self, config, number, platform, connection, address, platform_settings,
                 iflip_analog_input=255, iflip_eos_start_count=255):
        super().__init__(config, number)
        self.platform = platform
        self.connection = connection
        self.address = address
        self.platform_settings = platform_settings
        self.switch_rule = None
        self.switch_addresses = set()
        self.eos_switch_address = None
        self.iflip_analog_input = iflip_analog_input
        self.iflip_eos_start_count = iflip_eos_start_count

    def set_switch_rule(self, pulse_settings, hold_settings, auto_clear, can_cancel, delay_ms,
                        recycle_override=None):
        """Store and apply a hardware switch rule."""
        rule = (pulse_settings, hold_settings, auto_clear, can_cancel, delay_ms,
                recycle_override)
        if self.switch_rule and self.switch_rule != rule:
            raise AssertionError("Cannot configure different Copperhead rules on one solenoid")
        self.switch_rule = rule
        self.apply_switch_rule()

    def apply_switch_rule(self):
        """Restore the persistent hardware-rule configuration after a direct command."""
        if self.switch_rule:
            self.platform.configure_solenoid(self, *self.switch_rule)

    def remove_switch_rule(self):
        """Remove a hardware rule and restore the driver's default configuration."""
        self.switch_rule = None
        self.platform.configure_solenoid(self, None, None, auto_clear=True, can_cancel=False)

    def pulse(self, pulse_settings):
        self.platform.configure_solenoid(self, pulse_settings, None, auto_clear=True, can_cancel=False)
        self.platform.set_solenoid(self, True, cache_after=False)
        self.apply_switch_rule()

    def enable(self, pulse_settings, hold_settings):
        self.platform.configure_solenoid(self, pulse_settings, hold_settings, auto_clear=False, can_cancel=True)
        self.platform.set_solenoid(self, True)
        self.apply_switch_rule()

    def disable(self):
        self.platform.set_solenoid(self, False)

    def timed_enable(self, pulse_settings, hold_settings):
        self.enable(pulse_settings, hold_settings)
        duration = hold_settings.duration
        if duration is None:
            duration = self.config.default_timed_enable_ms
        self.platform.machine.clock.loop.call_later(duration / 1000.0, self.disable)

    def get_board_name(self):
        return "Copperhead {}".format(self.connection.board_number)
