"""Mission Pinball Framework platform for Copperhead hardware."""

import asyncio
import math

from mpf.core.platform import (
    DriverPlatform, LightsPlatform, MotorPlatform, ServoPlatform, StepperPlatform, SwitchPlatform,
)
from mpf.core.utility_functions import Util
from mpf.platforms.copperhead.copperhead_communicator import CopperheadCommunicator
from mpf.platforms.copperhead.copperhead_driver import CopperheadDriver
from mpf.platforms.copperhead.copperhead_light import CopperheadLight
from mpf.platforms.copperhead.copperhead_motor import CopperheadDCMotor
from mpf.platforms.copperhead.copperhead_protocol import (
    Command, bitmap_packet, interpolation_packet, packet, unpack_bits,
)
from mpf.platforms.copperhead.copperhead_servo import CopperheadServo
from mpf.platforms.copperhead.copperhead_stepper import CopperheadStepper
from mpf.platforms.copperhead.copperhead_switch import CopperheadSwitch


class CopperheadHardwarePlatform(LightsPlatform, SwitchPlatform, DriverPlatform, ServoPlatform,
                                 StepperPlatform, MotorPlatform):
    """Copperhead switch and solenoid platform."""

    __slots__ = ["config", "connections", "serial_connections", "switches", "drivers",
                 "switch_states", "_poll_tasks", "_gpio_commands", "_spi_commands", "spi_assignments",
                 "_iflip_analog_inputs", "_pending_interpolation", "_dc_motor_solenoids", "_dc_motors",
                 "_steppers", "_servos", "_neopixel_required_pixels", "_shutdown_task",
                 "_shutting_down", "_runtime_setup_started"]

    SPI_FUNCTIONS = {
        "motor_board": 2,
        "switch_matrix": 3,
        "lamp_matrix": 4,
        "8c16s": 5,
        "16s": 6,
    }
    SPI_ALLOCATIONS = {"explicit": 0, "next_spi0": 1, "next_spi1": 2}
    SPI_STATUS = {
        1: "invalid request", 2: "bad position", 3: "crosses SPI buses",
        4: "overlap", 5: "no space", 6: "assignment not found", 7: "assignment table full",
    }
    WING_NAMES = (
        "unused", "multi", "solenoid", "iflip", "iflip_adc",
        "switch_matrix_out1", "switch_matrix_out2", "switch_matrix_in1",
        "switch_matrix_in2", "lamp_matrix_col1", "lamp_matrix_col2",
        "lamp_matrix_row1", "lamp_matrix_row2", "spi",
    )
    MULTI_FUNCTIONS = {0x00: "switch", 0x40: "neopixel", 0x80: "servo", 0xc0: "i2c"}

    def __init__(self, machine):
        super().__init__(machine)
        self.features["tickless"] = True
        self.features["allow_empty_numbers"] = False
        self.features["max_pulse"] = 255
        self.features["hardware_eos_repulse"] = True
        self.config = self.machine.config_validator.validate_config(
            "copperhead", self.machine.config.get("copperhead", {}))
        self._configure_device_logging_and_debug("Copperhead", self.config)
        self.connections = {}
        self.serial_connections = set()
        self.switches = {}
        self.drivers = {}
        self.switch_states = {}
        self._poll_tasks = []
        self._gpio_commands = {}
        self._spi_commands = {}
        self.spi_assignments = {}
        self._iflip_analog_inputs = {}
        self._pending_interpolation = {}
        self._dc_motor_solenoids = set()
        self._dc_motors = {}
        self._steppers = {}
        self._servos = []
        self._neopixel_required_pixels = {}
        self._shutdown_task = None
        self._shutting_down = False
        self._runtime_setup_started = False

    async def initialize(self):
        """Connect all configured ports and discover their board numbers."""
        for port in self.config["ports"]:
            connection = CopperheadCommunicator(self, port, self.config["baud"])
            await connection.connect()
            self.serial_connections.add(connection)
        # Hardware scan stops after core/platform initialization. Normal game
        # startup runs this queued phase before DeviceManager (priority 5).
        self.machine.events.add_async_handler(
            "init_phase_1", self._initialize_runtime, priority=100)

    async def _initialize_runtime(self, **kwargs):
        """Apply runtime resources before MPF creates and validates devices."""
        del kwargs
        self._runtime_setup_started = True
        for connection in self.serial_connections:
            if connection.status or connection.fault:
                raise AssertionError("{} reports status {}, fault {}".format(
                    connection, connection.status, connection.fault))
            self._safe_clear_connection(connection)
        await self._configure_spi_functions()
        self._configure_cobrapin_auxiliary_outputs()
        for connection in self.serial_connections:
            self._safe_clear_lights(connection)
            self._log_runtime_summary(connection)
            self.log.info("Init of Copperhead board %s done", connection.board_number)

    async def start(self):
        """Start input polling."""
        await self._configure_neopixel_lengths()
        for connection in self.serial_connections:
            task = asyncio.create_task(self._poll(connection))
            task.add_done_callback(Util.raise_exceptions)
            self._poll_tasks.append(task)

    def stop(self):
        """Start a bounded asynchronous safe-output shutdown."""
        if not self._runtime_setup_started:
            for connection in tuple(self.serial_connections):
                connection.stop()
            self.serial_connections = set()
            return
        # Gate light updates synchronously. Device shutdown can leave delayed
        # light callbacks queued while the asynchronous hardware clear runs.
        self._shutting_down = True
        self._pending_interpolation = {}
        if self._shutdown_task and not self._shutdown_task.done():
            return
        if not self.serial_connections:
            return
        poll_tasks, self._poll_tasks = self._poll_tasks, []
        self._shutdown_task = self.machine.clock.loop.create_task(
            self._async_shutdown(poll_tasks))

    async def _async_shutdown(self, poll_tasks):
        """Flush safe output state before closing Copperhead USB ports."""
        self._pending_interpolation = {}
        # Let any command already sent by the poller consume its response. An
        # immediate cancellation can leave that response in the USB stream,
        # where the shutdown status barrier would mistake it for its own.
        if poll_tasks:
            await asyncio.gather(*poll_tasks, return_exceptions=True)
        for connection in tuple(self.serial_connections):
            messages = []
            board = connection.board_number
            self.log.info("Shutdown of Copperhead board %s started", board)
            self.debug_log("Copperhead board %s shutdown: begin", board)

            # Remove switch-to-solenoid rules before clearing command state so a
            # held switch cannot reassert an output during the remainder of shutdown.
            for driver in self.drivers.values():
                if driver.connection is connection and driver.switch_addresses:
                    for switch_address in sorted(driver.switch_addresses):
                        messages.append((
                            "remove hardware rule switch {} -> solenoid {}".format(
                                switch_address, driver.address),
                            packet(Command.SOLENOID_INPUT, switch_address,
                                   driver.address | 0x80)))
                    driver.switch_addresses.clear()
                    driver.switch_rule = None

            messages.append(("clear GPIO solenoids",
                             bitmap_packet(Command.KICK_SOLENOIDS, (), 0, 24)))
            messages.append(("clear SPI solenoids and DC motors",
                             bitmap_packet(Command.KICK_SPI_SOLENOIDS, (), 24, 104)))
            self._gpio_commands[board].clear()
            self._spi_commands[board].clear()

            for (stepper_board, _index), stepper in self._steppers.items():
                if stepper_board == board:
                    messages.append(("disable stepper {}".format(_index), stepper.prepare_stop()))
            for (motor_board, _index), motor in self._dc_motors.items():
                if motor_board == board:
                    # The SPI clear above drives both direction outputs low.
                    motor.prepare_stop()
                    self.debug_log("Copperhead board %s shutdown: cancel DC motor %s timer",
                                   board, _index)
            for servo in self._servos:
                if servo.connection is connection:
                    address, data, duration_ms = servo.prepare_stop()
                    messages.append(("disable servo GPIO {}".format(servo.gpio),
                                     interpolation_packet(address, duration_ms, data)))

            messages.append(("clear lamp matrix", interpolation_packet(0x2000, 0, bytes(64))))
            messages.append(("clear NeoPixels", interpolation_packet(0, 0, bytes(3 * 256 * 3))))
            try:
                await asyncio.wait_for(
                    connection.send_and_confirm(messages), timeout=0.9)
                self.debug_log("Copperhead board %s shutdown: status barrier confirmed", board)
                self.log.info(
                    "Shutdown of Copperhead board %s confirmed: status=%s fault=%s",
                    board, connection.status, connection.fault)
            except (OSError, ConnectionError, TimeoutError, ValueError, asyncio.TimeoutError) as exc:
                self.log.warning("Could not confirm safe shutdown for %s: %s", connection, exc)
            finally:
                self.debug_log("Copperhead board %s shutdown: close connection", board)
                connection.stop()
        self.serial_connections = set()

    @staticmethod
    def _safe_clear_connection(connection):
        """Best-effort clear of all authoritative solenoid command bits."""
        if not connection.writer:
            return
        try:
            connection.send(bitmap_packet(Command.KICK_SOLENOIDS, (), 0, 24))
            connection.send(bitmap_packet(Command.KICK_SPI_SOLENOIDS, (), 24, 104))
        except (OSError, RuntimeError):
            pass

    @staticmethod
    def _safe_clear_lights(connection):
        """Set every NeoPixel channel and lamp-matrix value to zero immediately."""
        if not connection.writer:
            return
        try:
            connection.send(interpolation_packet(0, 0, bytes(3 * 256 * 3)))
            connection.send(interpolation_packet(0x2000, 0, bytes(64)))
        except (OSError, RuntimeError):
            pass

    async def _configure_spi_functions(self):
        """Apply volatile SPI board functions and retain their resolved positions."""
        by_board = {}
        # CobraPin 3_00 has U9 at output byte 0 and four onboard input
        # registers (U15/U10/U17/U16) at SPI0 input bytes 0-3.
        for board, connection in self.connections.items():
            if (connection.hardware_model, connection.hardware_revision) == (1, (3, 0)):
                by_board[board] = [
                    {"function": "8c16s", "allocation": "explicit", "start_a": 0, "start_b": 0},
                    {"function": "16s", "allocation": "explicit", "start_a": 2, "start_b": 31},
                ]
        for entry in self.config["spi_functions"] or []:
            entries = by_board.setdefault(entry["board"], [])
            # Allow an existing config to repeat an onboard explicit assignment.
            if not any(all(entry[key] == item[key] for key in
                           ("function", "allocation", "start_a", "start_b"))
                       for item in entries):
                entries.append(entry)
        for board, entries in by_board.items():
            if board not in self.connections:
                self.raise_config_error(
                    "SPI functions reference Copperhead board {}, but connected boards are {}".format(
                        board, sorted(self.connections)), 6)
            connection = self.connections[board]
            clear = await connection.transact(Command.SET_SPI_BOARD_FUNCTION, 0, 31, 31)
            self._require_spi_success(board, "clear_all", clear)
            resolved = []
            for entry in entries:
                function = self.SPI_FUNCTIONS[entry["function"]]
                allocation = self.SPI_ALLOCATIONS[entry["allocation"]]
                config = function | (allocation << 5)
                result = await connection.transact(
                    Command.SET_SPI_BOARD_FUNCTION, config, entry["start_a"], entry["start_b"])
                self._require_spi_success(board, entry["function"], result)
                resolved.append({"function": entry["function"], "function_id": result[1],
                                 "start_a": result[2], "start_b": result[3]})
                self.log.info("Copperhead board %s assigned SPI function %s at A=%s B=%s",
                              board, entry["function"], result[2], result[3])
            self.spi_assignments[board] = resolved

    def _configure_cobrapin_auxiliary_outputs(self):
        """Provide onboard digital devices before MPF creates device collections."""
        for board, connection in self.connections.items():
            if (connection.hardware_model, connection.hardware_revision) != (1, (3, 0)):
                continue
            outputs = self.machine.config.setdefault("digital_outputs", {})
            for suffix, address in (("power_enable", 28), ("relay_30", 30), ("relay_31", 31)):
                name = "cobrapin_{}_{}".format(board, suffix)
                required = {"number": "{}-{}".format(board, address),
                            "platform": "copperhead", "type": "driver"}
                overrides = outputs.get(name, {})
                if not isinstance(overrides, dict):
                    self.raise_config_error("Digital output {} must be a dictionary".format(name), 23)
                for key, value in required.items():
                    if key in overrides and overrides[key] != value:
                        self.raise_config_error(
                            "Automatic digital output {} requires {}={}".format(name, key, value), 23)
                outputs[name] = dict(required, **overrides)
            # Configure Solenoid's special address selects a firmware-owned LED.
            connection.send(packet(Command.CONFIGURE_SOLENOID, 0xfe, 29, 0, 0, 0))

    def _require_spi_success(self, board, function, result):
        if result[0] != 0:
            raise AssertionError("Copperhead board {} rejected SPI function {}: status {} ({})".format(
                board, function, result[0], self.SPI_STATUS.get(result[0], "unknown")))

    def register_connection(self, connection):
        """Register a discovered controller using its board number as the mandatory key."""
        board = connection.board_number
        if board in self.connections:
            connection.stop()
            raise AssertionError("Duplicate Copperhead board number {} on {} and {}".format(
                board, self.connections[board].port, connection.port))
        self.connections[board] = connection
        self._gpio_commands[board] = set()
        self._spi_commands[board] = set()

    @classmethod
    def _wing_name(cls, value):
        return cls.WING_NAMES[value] if 0 <= value < len(cls.WING_NAMES) else "unknown({})".format(value)

    @staticmethod
    def _multi_addresses(connection, function):
        return [address for address, config in enumerate(connection.multi_config)
                if config != 0xff and config & 0xc0 == function]

    @staticmethod
    def _decode_neopixel_lengths(multi_config):
        """Return NeoPixel lengths in the firmware's ascending-GPIO output order."""
        return [((config & 0x0f) << 4) or 256 for config in multi_config
                if config != 0xff and config & 0xc0 == 0x40]

    @staticmethod
    def _neopixel_length_for_required_pixels(required_pixels):
        """Round an MPF output requirement to the firmware's 16-pixel units."""
        return min(256, max(16, ((required_pixels + 15) // 16) * 16))

    async def _configure_neopixel_lengths(self):
        """Clear full strips, then size each NeoPixel output for its MPF lights."""
        for connection in self.serial_connections:
            self._safe_clear_lights(connection)
            connection.status, connection.fault = await connection.transact(Command.GET_STATUS)
            if connection.status or connection.fault:
                raise AssertionError("{} reports status {}, fault {} while clearing NeoPixels".format(
                    connection, connection.status, connection.fault))
        # A 256-pixel WS2812 frame takes about 7.7ms. Let the original full
        # lengths transmit and latch before any output is shortened.
        if self.serial_connections:
            await asyncio.sleep(0.010)

        for connection in self.serial_connections:
            addresses = self._multi_addresses(connection, 0x40)
            for output, gpio in enumerate(addresses):
                required = self._neopixel_required_pixels.get(
                    (connection.board_number, output), 0)
                pixels = self._neopixel_length_for_required_pixels(required)
                encoded_length = 0 if pixels == 256 else pixels >> 4
                config = (connection.multi_config[gpio] & 0xf0) | encoded_length
                connection.send(packet(Command.CONFIGURE_INPUT, gpio, config))
                connection.multi_config[gpio] = config
                connection.neopixel_lengths[output] = pixels

                # Reconfiguration reapplies startup glow to every output. Clear
                # and latch the current lengths before another output shrinks.
                self._safe_clear_lights(connection)
                connection.status, connection.fault = await connection.transact(Command.GET_STATUS)
                if connection.status or connection.fault:
                    raise AssertionError(
                        "{} reports status {}, fault {} while sizing NeoPixel output {}".format(
                            connection, connection.status, connection.fault, output))
                await asyncio.sleep(0.010)

    def _log_runtime_summary(self, connection):
        assignments = self.spi_assignments.get(connection.board_number, [])
        functions = ", ".join("{}(A={},B={})".format(
            item["function"], item["start_a"], item["start_b"]) for item in assignments) or "none"
        self.log.info(
            "Copperhead board %s runtime resources: SPI functions=%s; NeoPixel outputs=%s; "
            "servo GPIOs=%s; GPIO solenoid wings=%s",
            connection.board_number, functions,
            connection.neopixel_lengths,
            self._multi_addresses(connection, 0x80),
            [wing for wing, value in enumerate(connection.wings) if value in (2, 3)])

    @staticmethod
    def _spi_switch_range(assignment):
        if assignment["function"] == "8c16s":
            first = 48 + assignment["start_b"] * 8
            return first, first + 15
        if assignment["function"] == "16s":
            first = 48 + assignment["start_a"] * 8
            return first, first + 15
        if assignment["function"] == "switch_matrix":
            return 192, 255
        return None

    @staticmethod
    def _spi_solenoid_range(assignment):
        if assignment["function"] != "8c16s":
            return None
        output_byte = assignment["start_a"]
        first = 24 + output_byte * 8 if output_byte < 8 else 88 + (output_byte - 8) * 8
        return first, first + 7

    def _parse_number(self, number, maximum):
        """Parse the required board-address number format."""
        try:
            board_text, address_text = str(number).split("-", 1)
            board = int(board_text, 0)
            address = int(address_text, 0)
        except (TypeError, ValueError):
            self.raise_config_error("Copperhead number '{}' must use board-address".format(number), 1)
        if board not in self.connections:
            self.raise_config_error("Copperhead board {} is not connected; connected boards: {}".format(
                board, sorted(self.connections)), 2)
        if not 0 <= address <= maximum:
            self.raise_config_error("Copperhead address {} is outside 0-{}".format(address, maximum), 3)
        return board, address, self.connections[board]

    def configure_switch(self, number, config, platform_config):
        del platform_config
        board, address, connection = self._parse_number(number, 255)
        if address < 48 and (connection.wings[address // 4] != 1 or
                             connection.multi_config[address] & 0xc0):
            self.raise_config_error(
                "Copperhead GPIO switch {} is unavailable: GPIO {} is not a MULTI switch input "
                "(wing {}, CC=0x{:02x})".format(
                    number, address, address // 4, connection.multi_config[address]), 21)
        if 48 <= address <= 191:
            input_byte = (address - 48) // 8
            owned = any(
                (item["function"] == "8c16s" and item["start_b"] <= input_byte < item["start_b"] + 2) or
                (item["function"] == "16s" and item["start_a"] <= input_byte < item["start_a"] + 2)
                for item in self.spi_assignments.get(board, []))
            if not owned:
                self.raise_config_error(
                    "Copperhead SPI switch {} is not owned by an 8C16S or 16S".format(number), 18)
        if address >= 192 and not any(
                item["function"] == "switch_matrix" for item in self.spi_assignments.get(board, [])):
            self.raise_config_error(
                "Copperhead matrix switch {} requires a Switch Matrix".format(number), 19)
        key = "{}-{}".format(board, address)
        switch = CopperheadSwitch(config, key, self, connection, address)
        self.switches[key] = switch
        self.switch_states.setdefault(key, False)
        return switch

    async def get_hw_switch_states(self):
        for connection in self.serial_connections:
            await self._read_switches(connection, notify=False)
        return dict(self.switch_states)

    @classmethod
    def get_coil_config_section(cls):
        """Return Copperhead-specific coil settings."""
        return "copperhead_coils"

    def configure_driver(self, config, number, platform_settings):
        board, address, connection = self._parse_number(number, 127)
        if (connection.hardware_model, connection.hardware_revision) == (1, (3, 0)):
            suffix = {28: "power_enable", 30: "relay_30", 31: "relay_31"}.get(address)
            if address == 29:
                self.raise_config_error("Copperhead output {} is reserved for the activity LED".format(number), 24)
            if suffix and (config is None or config.name != "cobrapin_{}_{}".format(board, suffix)):
                self.raise_config_error(
                    "Copperhead output {} is reserved for automatic digital output cobrapin_{}_{}".format(
                        number, board, suffix), 24)
        if "{}-{}".format(board, address) in self.drivers:
            self.raise_config_error("Copperhead output {} is configured more than once".format(number), 24)
        if (board, address) in self._dc_motor_solenoids:
            self.raise_config_error(
                "Copperhead solenoid {} is reserved by a configured DC motor".format(number), 12)
        if address < 24:
            wing = (address + 16) // 4
            if connection.wings[wing] not in (2, 3):
                self.raise_config_error(
                    "Copperhead GPIO solenoid {} is unavailable: physical GPIO {} wing {} is {}".format(
                        number, address + 16, wing, self._wing_name(connection.wings[wing])), 22)
        if address >= 24:
            output_byte = (address - 24) // 8 if address < 88 else 8 + (address - 88) // 8
            if not any(item["function"] == "8c16s" and item["start_a"] == output_byte
                       for item in self.spi_assignments.get(board, [])):
                self.raise_config_error(
                    "Copperhead SPI solenoid {} is not owned by an 8C16S".format(number), 20)
        analog_number = platform_settings.get("iflip_analog_input")
        if platform_settings.get("iflip", False) and analog_number is None:
            self.raise_config_error("iFlip solenoid {} requires iflip_analog_input".format(number), 7)
        if platform_settings.get("iflip", False) or analog_number is not None:
            if address >= 24:
                self.raise_config_error("iFlip cannot be enabled on SPI solenoid {}".format(number), 4)
            wing = (address + 16) // 4
            if connection.wings[wing] != 3:
                self.raise_config_error(
                    "Solenoid {} is not on an iFlip wing in the active Copperhead configuration".format(number), 5)
        analog_gpio = 255
        if analog_number is not None:
            analog_board, analog_gpio, analog_connection = self._parse_number(analog_number, 47)
            if analog_connection is not connection or analog_board != board:
                self.raise_config_error("iFlip analog input must be on the solenoid's Copperhead board", 8)
            if analog_gpio < 40 or connection.wings[analog_gpio // 4] != 4:
                self.raise_config_error(
                    "iFlip analog input {} must be GPIO 40-47 on an active IFLIP_ADC wing".format(
                        analog_number), 9)
            owner = self._iflip_analog_inputs.get((board, analog_gpio))
            if owner is not None and owner != address:
                self.raise_config_error(
                    "iFlip analog input {} is already assigned to solenoid {}-{}".format(
                        analog_number, board, owner), 10)
            self._iflip_analog_inputs[(board, analog_gpio)] = address
        key = "{}-{}".format(board, address)
        driver = CopperheadDriver(
            config, key, self, connection, address, platform_settings,
            analog_gpio, platform_settings["iflip_eos_start_count"])
        self.drivers[key] = driver
        self.configure_solenoid(driver, None, None,
                                auto_clear=not bool(config.default_hold_power), can_cancel=False)
        if address < 24 and connection.wings[(address + 16) // 4] == 3:
            connection.send(packet(
                Command.SET_IFLIP_INPUT, address, analog_gpio, 255,
                platform_settings["iflip_eos_start_count"]))
        return driver

    def parse_light_number_to_channels(self, number, subtype):
        """Expand a matrix lamp or NeoPixel into MPF light channels."""
        if subtype == "matrix":
            self._parse_number(number, 63)
            return [{"number": str(number)}]
        if subtype not in (None, "led", "neopixel"):
            raise AssertionError("Unknown Copperhead light subtype {}".format(subtype))
        try:
            board_text, output_text, pixel_text = str(number).split("-", 2)
            board, output, pixel = int(board_text, 0), int(output_text, 0), int(pixel_text, 0)
        except (TypeError, ValueError):
            raise AssertionError("Copperhead NeoPixel numbers use board-output-pixel")
        if board not in self.connections:
            raise AssertionError("Copperhead board {} is not connected".format(board))
        connection = self.connections[board]
        available_outputs = len(connection.neopixel_lengths)
        if not 0 <= output <= 2 or output >= available_outputs:
            raise AssertionError(
                "Copperhead NeoPixel output {} is unavailable on board {}; {} outputs are configured".format(
                    output, board, available_outputs))
        if not 0 <= pixel <= 255:
            raise AssertionError(
                "Copperhead board {} NeoPixel output {} pixel {} is out of range 0-255".format(
                    board, output, pixel))
        first = pixel * 3
        return [{"number": "{}-{}-{}".format(board, output, first + channel)} for channel in range(3)]

    def configure_light(self, number, subtype, config, platform_settings):
        """Configure one matrix or NeoPixel byte channel."""
        del config, platform_settings
        if subtype == "matrix":
            board, lamp, connection = self._parse_number(number, 63)
            if not any(item["function"] == "lamp_matrix" for item in self.spi_assignments.get(board, [])):
                raise AssertionError("Copperhead board {} has no assigned lamp matrix".format(board))
            return CopperheadLight(number, self, connection, 0x2000 + lamp, "matrix")
        if subtype not in (None, "led", "neopixel"):
            raise AssertionError("Unknown Copperhead light subtype {}".format(subtype))
        try:
            board_text, output_text, channel_text = str(number).split("-", 2)
            board, output, channel = int(board_text, 0), int(output_text, 0), int(channel_text, 0)
        except (TypeError, ValueError):
            raise AssertionError("Copperhead NeoPixel channels use board-output-channel")
        if board not in self.connections:
            raise AssertionError("Copperhead board {} is not connected".format(board))
        connection = self.connections[board]
        available_outputs = len(connection.neopixel_lengths)
        if not 0 <= output <= 2 or output >= available_outputs:
            raise AssertionError(
                "Copperhead NeoPixel output {} is unavailable on board {}; {} outputs are configured".format(
                    output, board, available_outputs))
        if not 0 <= channel <= 767:
            raise AssertionError(
                "Copperhead board {} NeoPixel output {} channel {} is out of range 0-767".format(
                    board, output, channel))
        required_pixels = channel // 3 + 1
        key = (board, output)
        self._neopixel_required_pixels[key] = max(
            required_pixels, self._neopixel_required_pixels.get(key, 0))
        return CopperheadLight(number, self, connection, output * 768 + channel,
                               "neopixel", output, channel)

    def queue_interpolation(self, connection, address, value, duration_ms):
        """Queue a light value until MPF's light synchronization point."""
        if self._shutting_down:
            return
        self._pending_interpolation.setdefault(connection, {})[address] = (value, duration_ms)

    def light_sync(self):
        """Coalesce adjacent light values with equal durations into interpolation packets."""
        if self._shutting_down:
            self._pending_interpolation = {}
            return
        pending, self._pending_interpolation = self._pending_interpolation, {}
        for connection, values in pending.items():
            addresses = sorted(values)
            index = 0
            while index < len(addresses):
                first = addresses[index]
                duration = values[first][1]
                data = bytearray((values[first][0],))
                index += 1
                while (index < len(addresses) and addresses[index] == first + len(data) and
                       values[addresses[index]][1] == duration):
                    data.append(values[addresses[index]][0])
                    index += 1
                self.send_interpolation(connection, first, data, duration)

    @staticmethod
    def send_interpolation(connection, address, data, duration_ms):
        """Send one interpolation packet."""
        connection.send(interpolation_packet(address, duration_ms, data))

    async def configure_servo(self, number, config):
        """Configure a servo addressed by board and physical GPIO."""
        del config
        board, gpio, connection = self._parse_number(number, 47)
        if not 16 <= gpio <= 39 or connection.wings[gpio // 4] != 1:
            self.raise_config_error(
                "Copperhead servo {} must be on a MULTI wing in wings 4-9".format(number), 11)
        if connection.multi_config[gpio] & 0xc0 != 0x80:
            self.raise_config_error(
                "Copperhead servo {} is unavailable: GPIO {} has CC=0x{:02x}, not servo function".format(
                    number, gpio, connection.multi_config[gpio]), 23)
        del board
        servo = CopperheadServo(self, connection, gpio)
        self._servos.append(servo)
        return servo

    @classmethod
    def get_stepper_config_section(cls):
        """Return Copperhead-specific stepper settings."""
        return "copperhead_steppers"

    def _motor_board_assignment(self, board, index):
        for assignment in self.spi_assignments.get(board, []):
            if assignment["function"] == "motor_board" and assignment["start_b"] == index:
                return assignment
        self.raise_config_error(
            "Copperhead board {} has no Motor Board with stepper index {}".format(board, index), 13)

    async def configure_stepper(self, number, config):
        """Configure a dynamically assigned Motor Board stepper."""
        board, index, connection = self._parse_number(number, 3)
        self._motor_board_assignment(board, index)
        if (board, index) in self._steppers:
            self.raise_config_error("Copperhead stepper {} is configured more than once".format(number), 14)
        home_board, home_address, home_connection = self._parse_number(config["home_switch"], 255)
        if home_board != board or home_connection is not connection:
            self.raise_config_error("Copperhead stepper home switch must be on the same board", 15)
        stepper = CopperheadStepper(
            self, connection, index, (home_board, home_address),
            config["microstep"], config["current_limit"], config["acceleration"])
        self._steppers[(board, index)] = stepper
        self.log.info("Copperhead board %s assigned stepper %s to Motor Board index %s",
                      board, number, index)
        return stepper

    async def configure_dc_motor(self, number, config):
        """Configure the bidirectional output pair on a Motor Board."""
        del config
        board, index, connection = self._parse_number(number, 3)
        assignment = self._motor_board_assignment(board, index)
        if (board, index) in self._dc_motors:
            self.raise_config_error("Copperhead DC motor {} is configured more than once".format(number), 16)
        output_byte = assignment["start_a"]
        first = 24 + output_byte * 8 if output_byte < 8 else 88 + (output_byte - 8) * 8
        outputs = ((board, first + 6), (board, first + 7))
        conflicts = ["{}-{}".format(*output) for output in outputs
                     if "{}-{}".format(*output) in self.drivers]
        if conflicts:
            self.raise_config_error(
                "Copperhead DC motor {} conflicts with coils {}".format(number, ", ".join(conflicts)), 17)
        self._dc_motor_solenoids.update(outputs)
        motor = CopperheadDCMotor(self, connection, first + 6, first + 7)
        self._dc_motors[(board, index)] = motor
        self.log.info("Copperhead board %s assigned DC motor %s to solenoids %s and %s",
                      board, number, first + 6, first + 7)
        return motor

    @staticmethod
    def _power(value, maximum=127):
        return max(0, min(maximum, round(float(value or 0) * maximum)))

    def configure_solenoid(self, driver, pulse_settings, hold_settings, auto_clear, can_cancel,
                           delay_ms=None, recycle_override=None):
        pulse_ms = driver.config.default_pulse_ms if pulse_settings is None else pulse_settings.duration
        pulse_power = driver.config.default_pulse_power if pulse_settings is None else pulse_settings.power
        hold_power = driver.config.default_hold_power if hold_settings is None else hold_settings.power
        if delay_ms is not None:
            if hold_settings and hold_settings.power:
                raise AssertionError("Copperhead delayed pulses cannot use hold power")
            if not 0 <= delay_ms <= 255:
                raise AssertionError("Copperhead pulse delay must be between 0 and 255ms")
        flags = ((0x80 if auto_clear else 0) |
                 (0x40 if delay_ms is not None else 0) |
                 (0x20 if can_cancel else 0) |
                 (0x10 if driver.platform_settings.get("high_speed", False) else 0) |
                 (0x08 if driver.platform_settings.get("iflip", False) else 0) |
                 (driver.platform_settings.get("recycle", 0) if recycle_override is None
                  else recycle_override))
        power_maximum = 255 if driver.platform_settings.get("iflip", False) else 127
        driver.connection.send(packet(Command.CONFIGURE_SOLENOID, driver.address, flags,
                                      delay_ms if delay_ms is not None else self._power(hold_power, power_maximum),
                                      max(0, min(255, pulse_ms)),
                                      self._power(pulse_power, power_maximum)))

    def set_solenoid(self, driver, active, cache_after=None):
        values = self._gpio_commands if driver.address < 24 else self._spi_commands
        commanded = values[driver.connection.board_number]
        if active:
            commanded.add(driver.address)
        else:
            commanded.discard(driver.address)
        if driver.address < 24:
            message = bitmap_packet(Command.KICK_SOLENOIDS, commanded, 0, 24)
        else:
            message = bitmap_packet(Command.KICK_SPI_SOLENOIDS, commanded, 24, 104)
        driver.connection.send(message)
        if cache_after is False:
            commanded.discard(driver.address)

    def _set_rule(self, enable_switch, coil, auto_clear, can_cancel, hold, delay_ms=None,
                  recycle_override=None):
        switch = enable_switch.hw_switch
        driver = coil.hw_driver
        if switch.address == 255:
            raise AssertionError("Copperhead matrix switch 255 cannot be assigned to a solenoid")
        if switch.connection is not driver.connection:
            raise AssertionError("Copperhead hardware rules require switch and solenoid on the same board")
        driver.set_switch_rule(coil.pulse_settings, coil.hold_settings if hold else None,
                               auto_clear, can_cancel, delay_ms, recycle_override)
        if switch.address not in driver.switch_addresses:
            driver.connection.send(packet(Command.SOLENOID_INPUT, switch.address, driver.address))
            driver.switch_addresses.add(switch.address)

    def clear_hw_rule(self, switch, coil):
        hw_switch = switch.hw_switch
        driver = coil.hw_driver
        if hw_switch.address in driver.switch_addresses:
            driver.connection.send(packet(
                Command.SOLENOID_INPUT, hw_switch.address, driver.address | 0x80))
            driver.switch_addresses.remove(hw_switch.address)
        if driver.eos_switch_address == hw_switch.address:
            driver.connection.send(packet(
                Command.SET_IFLIP_INPUT, driver.address, driver.iflip_analog_input,
                255, driver.iflip_eos_start_count))
            driver.eos_switch_address = None
        if not driver.switch_addresses and driver.switch_rule is not None:
            driver.remove_switch_rule()
            driver.disable()

    def set_pulse_on_hit_rule(self, enable_switch, coil):
        self._set_rule(enable_switch, coil, auto_clear=True, can_cancel=False, hold=False)

    def set_delayed_pulse_on_hit_rule(self, enable_switch, coil, delay_ms):
        if not 0 < delay_ms <= 255:
            raise AssertionError("Copperhead delayed pulse rules require a delay from 1 to 255ms")
        self._set_rule(enable_switch, coil, auto_clear=True, can_cancel=False, hold=False,
                       delay_ms=int(delay_ms))

    def set_pulse_on_hit_and_release_rule(self, enable_switch, coil):
        self._set_rule(enable_switch, coil, auto_clear=True, can_cancel=True, hold=False)

    def set_pulse_on_hit_and_enable_and_release_rule(self, enable_switch, coil):
        self._set_rule(enable_switch, coil, auto_clear=False, can_cancel=True, hold=True)

    def set_pulse_on_hit_and_release_and_disable_rule(self, enable_switch, eos_switch, coil, repulse_settings):
        self._set_eos_rule(enable_switch, eos_switch, coil, repulse_settings, hold=False)

    def set_pulse_on_hit_and_enable_and_release_and_disable_rule(self, enable_switch, eos_switch,
                                                                 coil, repulse_settings):
        self._set_eos_rule(enable_switch, eos_switch, coil, repulse_settings, hold=True)

    def _set_eos_rule(self, enable_switch, eos_switch, coil, repulse_settings, hold):
        driver = coil.hw_driver
        eos = eos_switch.hw_switch
        if not driver.platform_settings.get("iflip", False):
            raise AssertionError("Copperhead physical EOS rules require an iFlip solenoid")
        if eos.address == 255:
            raise AssertionError("Copperhead switch 255 cannot be used as an EOS switch")
        if eos.connection is not driver.connection:
            raise AssertionError("Copperhead EOS switch and solenoid must be on the same board")
        pulse_ms = coil.pulse_settings.duration
        if pulse_ms <= 0:
            raise AssertionError("Copperhead physical EOS rules require a nonzero kick time")
        eos_repulse = bool(repulse_settings and repulse_settings.enable_repulse)
        recycle = driver.platform_settings.get("recycle", 0)
        if eos_repulse:
            recycle = min(7, max(recycle, math.ceil(
                repulse_settings.debounce_ms / pulse_ms)))
        self._set_rule(enable_switch, coil, auto_clear=not hold,
                       can_cancel=True, hold=hold, recycle_override=recycle)
        control = driver.address | 0x80 | (0x40 if eos_repulse else 0)
        driver.connection.send(packet(
            Command.SET_IFLIP_INPUT, control, driver.iflip_analog_input,
            eos.address, driver.iflip_eos_start_count))
        driver.eos_switch_address = eos.address

    async def _poll(self, connection):
        interval = 1.0 / self.config["poll_hz"]
        next_status = self.machine.clock.get_time() + 1.0
        try:
            while not self._shutting_down:
                await self._read_switches(connection, notify=True)
                if self._shutting_down:
                    return
                if self.machine.clock.get_time() >= next_status:
                    connection.status, connection.fault = await connection.transact(Command.GET_STATUS)
                    next_status += 1.0
                    if connection.status:
                        reason = {1: "watchdog reboot", 2: "invalid wing configuration",
                                  3: "NeoPixel initialization", 4: "SPI timeout",
                                  5: "control-loop overrun",
                                  6: "connection watchdog"}.get(connection.fault, "unknown fault")
                        self.machine.stop("Copperhead board {} fault {}: {}".format(
                            connection.board_number, connection.fault, reason))
                        return
                if not self._shutting_down:
                    await asyncio.sleep(interval)
        except asyncio.CancelledError:
            raise
        except (OSError, EOFError, TimeoutError, ValueError) as exc:
            if self._shutting_down:
                return
            self.log.error("Copperhead communication failure on %s: %s", connection.port, exc)
            self._safe_clear_connection(connection)
            connection.stop()
            self.machine.stop("Copperhead communication failure on {}: {}".format(connection.port, exc))

    async def _read_switches(self, connection, notify):
        """Read and merge GPIO, SPI, and matrix inputs into flattened addresses."""
        gpio = unpack_bits(await connection.transact(Command.READ_INPUTS), 48)
        spi = unpack_bits(await connection.transact(Command.READ_SPI_INPUTS), 144)
        matrix = unpack_bits(await connection.transact(Command.READ_MATRIX_INPUTS), 64)
        states = gpio + spi + matrix
        board = connection.board_number
        for address, state in enumerate(states):
            key = "{}-{}".format(board, address)
            if key in self.switches and self.switch_states[key] != state:
                self.switch_states[key] = state
                if notify:
                    self.machine.switch_controller.process_switch_by_num(
                        state=state, num=key, platform=self)

    def get_info_string(self):
        lines = ["Copperhead Hardware (read-only discovery)", "-----------------------------------------"]
        for board, connection in sorted(self.connections.items()):
            lines.extend(("", "Board {} — {} at {} baud".format(
                board, connection.port, connection.baud)))
            lines.append("  Firmware: {}".format(
                ".".join(str(value) for value in connection.firmware_version)))
            lines.append("  Hardware model: {}; revision: {}.{}".format(
                connection.hardware_model, *connection.hardware_revision))
            lines.append("  Status: {} ({})".format(
                "healthy" if connection.status == 0 else "error", connection.status))
            lines.append("  Fault: {} ({})".format(
                "none" if connection.fault == 0 else "active", connection.fault))

            lines.extend(("", "Wings:"))
            spi_wings = []
            for wing, value in enumerate(connection.wings):
                first_gpio = wing * 4
                name = self._wing_name(value).upper()
                detail = "GPIO {}-{}".format(first_gpio, first_gpio + 3)
                if value == 13:
                    instance = (wing >> 1) & 1
                    detail = "SPI{}".format(instance)
                    spi_wings.append((wing, instance))
                elif value in (2, 3):
                    detail = "solenoids {}-{}".format(first_gpio - 16, first_gpio - 13)
                lines.append("  {:2d} {:<12} {}".format(wing, name, detail))

            lines.extend(("", "MULTI GPIOs:"))
            neopixel_output = 0
            switches = []
            servos = []
            i2c_pairs = []
            for gpio, config in enumerate(connection.multi_config):
                if config == 0xff:
                    continue
                function = config & 0xc0
                if function == 0x00:
                    level = (config >> 3) & 7
                    samples = (1, 2, 3, 5, 9, 17, 33, 65)[level]
                    active_high = bool(config & 0x04)
                    meaning = "Switch, active-{}/pull-{}, debounce {} ({} sample{})".format(
                        "high" if active_high else "low", "down" if active_high else "up",
                        level, samples, "" if samples == 1 else "s")
                    switches.append(gpio)
                elif function == 0x40:
                    pixels = (config & 0x0f) << 4 or 256
                    glow = (config >> 4) & 3
                    meaning = "NeoPixel {}, {} pixels, glow {}".format(
                        neopixel_output, pixels, glow)
                    neopixel_output += 1
                elif function == 0x80:
                    position = (config & 0x3f) << 2
                    meaning = "Servo, default {} ({:.2f} ms)".format(position, position / 100.0)
                    servos.append(gpio)
                else:
                    pair_first = gpio & 0xfe
                    pair = "{}/{}".format(pair_first, pair_first + 1)
                    complete = (pair_first + 1 < len(connection.multi_config) and
                                connection.multi_config[pair_first] != 0xff and
                                connection.multi_config[pair_first + 1] != 0xff and
                                connection.multi_config[pair_first] & 0xc0 == 0xc0 and
                                connection.multi_config[pair_first + 1] & 0xc0 == 0xc0)
                    meaning = "I2C {}, pair {} {}".format(
                        "SDA" if gpio % 2 == 0 else "SCL", pair,
                        "complete" if complete else "incomplete")
                    if complete and pair not in i2c_pairs:
                        i2c_pairs.append(pair)
                lines.append("  GPIO {:2d}  CC={:02X}  {}".format(gpio, config, meaning))

            lines.extend(("", "Potential SPI capacity:"))
            total_inputs = 0
            total_outputs = 0
            for wing, instance in spi_wings:
                if instance == 0:
                    inputs, outputs = 96, 64
                    input_range, output_range, unused = "48-143", "24-87", 32
                else:
                    inputs, outputs = 48, 40
                    input_range, output_range, unused = "144-191", "88-127", 8
                total_inputs += inputs
                total_outputs += outputs
                lines.append("  SPI{}  wing {:2d}  inputs {} ({})  outputs {} ({})".format(
                    instance, wing, inputs, input_range, outputs, output_range))
                lines.append("                 transfer {} bits; {} unused output bits shifted first".format(
                    inputs, unused))

            standard_solenoids = []
            iflip_solenoids = []
            for wing, value in enumerate(connection.wings):
                if value == 2:
                    standard_solenoids.extend(range(wing * 4 - 16, wing * 4 - 12))
                elif value == 3:
                    iflip_solenoids.extend(range(wing * 4 - 16, wing * 4 - 12))
            lines.extend(("", "Summary:"))
            lines.append("  Switch GPIOs:       {}".format(self._format_numbers(switches)))
            lines.append("  Standard solenoids: {}".format(self._format_numbers(standard_solenoids)))
            lines.append("  iFlip solenoids:    {}".format(self._format_numbers(iflip_solenoids)))
            lines.append("  NeoPixel outputs:   {}".format(neopixel_output))
            lines.append("  Servo GPIOs:        {}".format(self._format_numbers(servos)))
            lines.append("  I2C pairs:          {}".format(", ".join(i2c_pairs) or "none"))
            lines.append("  Potential SPI:      {} inputs, {} outputs".format(
                total_inputs, total_outputs))
            if not self.machine.options.get("hardware_scan", False):
                assignments = self.spi_assignments.get(board, [])
                switch_ranges = [self._spi_switch_range(item) for item in assignments]
                solenoid_ranges = [self._spi_solenoid_range(item) for item in assignments]
                lines.append("  Runtime configuration: SPI switch ranges {}; SPI solenoid ranges {}; "
                             "switch matrix {}; lamp matrix {}; motor boards {}".format(
                                 [item for item in switch_ranges if item],
                                 [item for item in solenoid_ranges if item],
                                 any(item["function"] == "switch_matrix" for item in assignments),
                                 any(item["function"] == "lamp_matrix" for item in assignments),
                                 [(item["start_a"], item["start_b"]) for item in assignments
                                  if item["function"] == "motor_board"]))
        return "\n".join(lines)

    @staticmethod
    def _format_numbers(values):
        if not values:
            return "none"
        ranges = []
        start = previous = values[0]
        for value in values[1:]:
            if value == previous + 1:
                previous = value
                continue
            ranges.append(str(start) if start == previous else "{}-{}".format(start, previous))
            start = previous = value
        ranges.append(str(start) if start == previous else "{}-{}".format(start, previous))
        return ", ".join(ranges)
