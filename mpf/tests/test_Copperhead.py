"""Integration tests for the Copperhead MPF platform."""

import asyncio
from types import SimpleNamespace

from serial import SerialException

from mpf.platforms.copperhead.copperhead import CopperheadHardwarePlatform
from mpf.platforms.copperhead.copperhead_protocol import Command, crc8, interpolation_packet, packet
from mpf.tests.MpfTestCase import MpfTestCase, expect_startup_error
from mpf.tests.loop import MockSerial


REQUEST_LENGTHS = {
    Command.GET_BOARD_NUMBER: 3,
    Command.GET_VERSION: 3,
    Command.KICK_SOLENOIDS: 6,
    Command.READ_INPUTS: 3,
    Command.GET_MULTI_CONFIG: 3,
    Command.GET_WING_CONFIG: 3,
    Command.CONFIGURE_SOLENOID: 8,
    Command.CONFIGURE_INPUT: 5,
    Command.SET_IFLIP_INPUT: 7,
    Command.STEPPER_COMMAND: 7,
    Command.SOLENOID_INPUT: 5,
    Command.READ_MATRIX_INPUTS: 3,
    Command.READ_SPI_INPUTS: 3,
    Command.GET_STATUS: 3,
    Command.KICK_SPI_SOLENOIDS: 16,
    Command.SET_SPI_BOARD_FUNCTION: 6,
    Command.GET_STEPPER_POSITION: 4,
}


class MockCopperheadSerial(MockSerial):
    """Small stateful Copperhead USB CDC emulator."""

    def __init__(self, board_number=20):
        super().__init__()
        self.board_number = board_number
        self.hardware_model = 0
        self.hardware_revision = (0, 0)
        self.wings = [1, 13, 0, 0, 1, 1, 2, 3, 0, 0, 4, 13]
        self.queue = []
        self.messages = []
        self.spi_results = {
            0x00: (0, 0, 31, 31),
            0x25: (0, 5, 0, 0),
            0x43: (0, 3, 8, 12),
            0x24: (0, 4, 1, 31),
            0x22: (0, 2, 3, 0),
        }
        self.status_result = (0, 0)
        self.stepper_positions = [0] * 4
        self.drop_command = None
        self.corrupt_command = None
        self.delay_command = None
        self.delayed_responses = []
        self.response_chunks = None
        self.partial_command = None
        self.partial_response_bytes = 0
        self.disconnect_command = None
        self.disconnect_pending = False
        self.fail_write_command = None
        self.multi_config_response = [0xFF] * 48
        self.multi_config_response[0:4] = [0xC0, 0xC0, 0x50, 0x50]
        self.multi_config_response[16:24] = [0x50, 0x80, 0, 0, 0, 0, 0, 0]

    def read_ready(self):
        return bool(self.queue) or self.disconnect_pending

    def write_ready(self):
        return True

    def read(self, length):
        if not self.queue:
            if self.disconnect_pending:
                self.disconnect_pending = False
                raise SerialException("injected Copperhead USB disconnect")
            return b""
        data = self.queue.pop(0)
        if len(data) > length:
            self.queue.insert(0, data[length:])
            return data[:length]
        return data

    def _enqueue_response(self, response):
        if self.response_chunks:
            offset = 0
            chunks = ([self.response_chunks] if isinstance(self.response_chunks, int)
                      else self.response_chunks)
            chunk_index = 0
            while offset < len(response):
                size = chunks[min(chunk_index, len(chunks) - 1)]
                self.queue.append(response[offset:offset + size])
                offset += size
                chunk_index += 1
        else:
            self.queue.append(response)

    def release_delayed_responses(self):
        """Make responses held by delay_command readable."""
        for response in self.delayed_responses:
            self._enqueue_response(response)
        self.delayed_responses = []

    def write(self, data):
        original_length = len(data)
        while data:
            if len(data) < 2:
                raise AssertionError("Incomplete Copperhead request")
            command = Command(data[1])
            if command == self.fail_write_command:
                raise OSError("injected Copperhead USB write failure")
            length = (9 + int.from_bytes(data[4:6], "big")
                      if command == Command.INTERPOLATE else REQUEST_LENGTHS[command])
            message, data = data[:length], data[length:]
            if len(message) != length or message[-1] != crc8(message[:-1]):
                raise AssertionError("Invalid Copperhead request: {}".format(message.hex(" ")))
            self.messages.append(message)
            self._respond(command, message)
        return original_length

    def _respond(self, command, message):
        if command == self.drop_command:
            return
        response = None
        if command == Command.GET_BOARD_NUMBER:
            response = packet(command, *self.board_number.to_bytes(4, "big"))
        elif command == Command.GET_VERSION:
            response = packet(command, 0, 0, 0, 14,
                              *self.hardware_model.to_bytes(2, "big"), *self.hardware_revision)
        elif command == Command.GET_WING_CONFIG:
            response = packet(command, *self.wings)
        elif command == Command.GET_MULTI_CONFIG:
            response = packet(command, *self.multi_config_response)
        elif command == Command.GET_STATUS:
            response = packet(command, *self.status_result)
        elif command == Command.READ_INPUTS:
            response = packet(command, *(1 << 18).to_bytes(6, "little"))
        elif command == Command.READ_SPI_INPUTS:
            response = packet(command, *bytes(18))
        elif command == Command.READ_MATRIX_INPUTS:
            response = packet(command, *bytes(8))
        elif command == Command.SET_SPI_BOARD_FUNCTION:
            result = self.spi_results[message[2]]
            response = packet(command, *result)
        elif command == Command.GET_STEPPER_POSITION:
            index = message[2]
            response = packet(command, index,
                              *self.stepper_positions[index].to_bytes(4, "big", signed=True))
        elif command == Command.INTERPOLATE:
            address = int.from_bytes(message[2:4], "big")
            length = int.from_bytes(message[4:6], "big")
            if 0x4000 <= address <= 0x400c and address % 4 == 0 and length == 4:
                index = (address - 0x4000) // 4
                self.stepper_positions[index] = int.from_bytes(
                    message[8:12], "big", signed=True)
        if response is None:
            return
        if command == self.partial_command:
            response = response[:self.partial_response_bytes]
        if command == self.disconnect_command:
            response = response[:self.partial_response_bytes or 1]
            self.disconnect_pending = True
        if command == self.corrupt_command:
            corrupt = bytearray(response)
            corrupt[-1] ^= 1
            response = bytes(corrupt)
        if command == self.delay_command:
            self.delayed_responses.append(response)
        else:
            self._enqueue_response(response)


class TestCopperhead(MpfTestCase):
    """Exercise Copperhead through MPF's mocked serial event loop."""

    def get_machine_path(self):
        return "tests/machine_files/copperhead/"

    def get_config_file(self):
        return "config.yaml"

    def get_platform(self):
        return False

    def _mock_loop(self):
        self.clock.mock_serial("com1", self.serial)

    def setUp(self):
        self.serial = MockCopperheadSerial()
        super().setUp()

    def test_initialization_switches_and_driver(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.assertIsInstance(platform, CopperheadHardwarePlatform)
        self.assertEqual(20, next(iter(platform.connections)))
        self.assertEqual([16, 64, 256], platform.connections[20].neopixel_lengths)
        neopixel_configs = [message[2:4] for message in self.serial.messages
                            if message[1] == Command.CONFIGURE_INPUT]
        self.assertEqual([bytes((2, 0x51)), bytes((3, 0x54)), bytes((16, 0x50))],
                         neopixel_configs)
        self.assertEqual([
            {"function": "8c16s", "function_id": 5, "start_a": 0, "start_b": 0},
            {"function": "switch_matrix", "function_id": 3, "start_a": 8, "start_b": 12},
            {"function": "lamp_matrix", "function_id": 4, "start_a": 1, "start_b": 31},
            {"function": "motor_board", "function_id": 2, "start_a": 3, "start_b": 0},
        ], platform.spi_assignments[20])
        self.assertTrue(self.machine.switches["s_copperhead"].state)

        self.machine.coils["c_copperhead"].pulse()
        self.machine.lights["l_matrix"].on(brightness=0.5)
        self.machine.lights["l_neopixel"].color("112233")
        self.machine.servos["servo_test"].go_to_position(200 / 255)
        self.machine.steppers["stepper_test"].hw_stepper.move_rel_pos(100, speed=1000)
        self.machine.dc_motors["motor_test"].pulse(duration=.02, power=.5)
        self.advance_time_and_run(.01)

        commands = [message[1] for message in self.serial.messages]
        self.assertIn(Command.GET_BOARD_NUMBER, commands)
        self.assertIn(Command.GET_VERSION, commands)
        self.assertIn(Command.GET_WING_CONFIG, commands)
        self.assertIn(Command.GET_MULTI_CONFIG, commands)
        self.assertIn(Command.GET_STATUS, commands)
        self.assertIn(Command.SET_SPI_BOARD_FUNCTION, commands)
        self.assertIn(Command.CONFIGURE_SOLENOID, commands)
        self.assertIn(Command.KICK_SOLENOIDS, commands)
        self.assertIn(Command.SET_IFLIP_INPUT, commands)
        self.assertIn(Command.INTERPOLATE, commands)
        self.assertIn(Command.STEPPER_COMMAND, commands)
        self.assertEqual((0, 0, 0, 14), platform.connections[20].firmware_version)
        self.assertEqual(0, platform.connections[20].hardware_model)
        self.assertEqual((0, 0), platform.connections[20].hardware_revision)
        info = platform.get_info_string()
        self.assertIn("Firmware: 0.0.0.14", info)
        self.assertIn("GPIO 17  CC=80  Servo, default 0 (0.00 ms)", info)
        self.assertIn("SPI0  wing  1  inputs 96 (48-143)  outputs 64 (24-87)", info)
        self.assertIn("Potential SPI:      144 inputs, 104 outputs", info)

        final_matrix_light = platform.configure_light("20-63", "matrix", None, None)
        self.assertEqual("20-64", final_matrix_light.get_successor_number())
        with self.assertRaisesRegex(AssertionError, "outside 0-63"):
            platform.configure_light("20-64", "matrix", None, None)

        stepper_move = next(message for message in self.serial.messages
                            if message[1] == Command.STEPPER_COMMAND and message[3] & 1)
        self.assertEqual(bytes((0, 0xF1, 0x32, 18)), stepper_move[2:6])

        iflip = next(message for message in self.serial.messages if message[1] == Command.SET_IFLIP_INPUT)
        self.assertEqual(bytes((12, 40, 255, 8)), iflip[2:6])
        iflip_config = next(message for message in self.serial.messages
                            if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 12)
        # With iFlip enabled, KP carries the current threshold. Firmware drives
        # the physical kick at maximum power while regulating to this value.
        self.assertEqual(128, iflip_config[6])

        interpolations = [message for message in self.serial.messages if message[1] == Command.INTERPOLATE]
        self.assertTrue(any(message[2:4] == bytes((0, 0)) and
                            int.from_bytes(message[4:6], "big") == 2304 and
                            set(message[8:-1]) <= {0} for message in interpolations))
        self.assertTrue(any(message[2:4] == bytes((0x20, 0)) and
                            int.from_bytes(message[4:6], "big") == 64 and
                            set(message[8:-1]) <= {0} for message in interpolations))
        self.assertTrue(any(message[2:6] == bytes.fromhex("00 03 00 03") for message in interpolations))
        self.assertTrue(any(message[2:4] == bytes.fromhex("20 00") for message in interpolations))
        self.assertTrue(any(message[2:4] == bytes.fromhex("30 11") for message in interpolations))
        self.assertTrue(any(message[2:4] == bytes.fromhex("40 00") for message in interpolations))

        motor_config = next(message for message in self.serial.messages
                            if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 54)
        self.assertEqual(bytes((0x22, 64, 32, 64)), motor_config[3:7])
        self.advance_time_and_run(.02)
        self.assertNotIn(54, platform._spi_commands[20])

    def test_stepper_waits_for_completion_before_next_target(self):
        stepper = self.machine.steppers["stepper_test"]
        self.advance_time_and_run(.01)
        self.serial.delay_command = Command.GET_STEPPER_POSITION
        stepper.move_to_position(1000, speed=1000)
        self.advance_time_and_run(.25)
        stepper.move_to_position(500, speed=1000)
        self.advance_time_and_run(.01)

        moves = [message for message in self.serial.messages
                 if message[1] == Command.INTERPOLATE and message[2:4] == bytes.fromhex("40 00")]
        self.assertEqual(1, len(moves))
        self.assertTrue(self.serial.delayed_responses)

        self.serial.delay_command = None
        self.serial.release_delayed_responses()
        self.advance_time_and_run(.02)
        moves = [message for message in self.serial.messages
                 if message[1] == Command.INTERPOLATE and message[2:4] == bytes.fromhex("40 00")]
        self.assertEqual(2, len(moves))
        self.assertEqual(1000, int.from_bytes(moves[-2][8:12], "big", signed=True))
        self.assertEqual(500, int.from_bytes(moves[-1][8:12], "big", signed=True))
        self.assertTrue(any(message[1] == Command.GET_STEPPER_POSITION
                            for message in self.serial.messages))

    def test_stepper_command_modes_and_zero_time_move(self):
        hardware = self.machine.steppers["stepper_test"].hw_stepper
        self.serial.messages.clear()

        hardware.home("clockwise")
        hardware.stop()
        hardware.move_rel_pos(-10)

        commands = [message for message in self.serial.messages
                    if message[1] == Command.STEPPER_COMMAND]
        self.assertEqual(bytes((0, 0x17, 0x32, 18)), commands[0][2:6])
        self.assertEqual(bytes((0, 0xF0, 0x32, 18)), commands[1][2:6])
        self.assertEqual(bytes((0, 0xF1, 0x32, 18)), commands[2][2:6])

        move = next(message for message in self.serial.messages
                    if message[1] == Command.INTERPOLATE)
        self.assertEqual(bytes((0, 0)), move[6:8])
        self.assertEqual(-10, int.from_bytes(move[8:12], "big", signed=True))

    def test_iflip_physical_eos_rule_and_recycle_restore(self):
        platform = self.machine.hardware_platforms["copperhead"]
        driver = self.machine.coils["c_iflip"].hw_driver
        enable_switch = SimpleNamespace(
            hw_switch=self.machine.switches["s_copperhead"].hw_switch)
        eos_switch = SimpleNamespace(
            hw_switch=self.machine.switches["s_copperhead_eos"].hw_switch)
        coil = SimpleNamespace(
            hw_driver=driver,
            pulse_settings=SimpleNamespace(duration=32, power=0.5),
            hold_settings=SimpleNamespace(power=0.25))
        repulse = SimpleNamespace(enable_repulse=True, debounce_ms=500)
        self.serial.messages.clear()

        platform.set_pulse_on_hit_and_enable_and_release_and_disable_rule(
            enable_switch, eos_switch, coil, repulse)

        self.assertTrue(platform.features["hardware_eos_repulse"])
        solenoid_config = next(
            message for message in self.serial.messages
            if message[1] == Command.CONFIGURE_SOLENOID)
        self.assertEqual(7, solenoid_config[3] & 0x07)
        eos_config = next(
            message for message in self.serial.messages
            if message[1] == Command.SET_IFLIP_INPUT)
        self.assertEqual(bytes((0xCC, 40, 19, 8)), eos_config[2:6])

        platform.clear_hw_rule(enable_switch, coil)
        platform.clear_hw_rule(eos_switch, coil)
        eos_configs = [message for message in self.serial.messages
                       if message[1] == Command.SET_IFLIP_INPUT]
        self.assertEqual(bytes((12, 40, 255, 8)), eos_configs[-1][2:6])
        solenoid_configs = [message for message in self.serial.messages
                            if message[1] == Command.CONFIGURE_SOLENOID]
        self.assertEqual(0, solenoid_configs[-1][3] & 0x07)

    def test_multiple_switch_rules_on_one_coil_are_tracked_independently(self):
        platform = self.machine.hardware_platforms["copperhead"]
        driver = self.machine.coils["c_copperhead"].hw_driver
        coil = SimpleNamespace(
            hw_driver=driver,
            pulse_settings=SimpleNamespace(duration=32, power=1.0),
            hold_settings=SimpleNamespace(power=0.25))
        first_switch = SimpleNamespace(
            hw_switch=self.machine.switches["s_copperhead"].hw_switch)
        second_switch = SimpleNamespace(
            hw_switch=self.machine.switches["s_copperhead_eos"].hw_switch)
        self.serial.messages.clear()

        platform.set_pulse_on_hit_rule(first_switch, coil)
        platform.set_pulse_on_hit_rule(second_switch, coil)
        platform.set_pulse_on_hit_rule(first_switch, coil)

        associations = [message for message in self.serial.messages
                        if message[1] == Command.SOLENOID_INPUT]
        self.assertEqual([
            packet(Command.SOLENOID_INPUT, 18, driver.address),
            packet(Command.SOLENOID_INPUT, 19, driver.address),
        ], associations)
        self.assertEqual({18, 19}, driver.switch_addresses)

        self.serial.messages.clear()
        platform.clear_hw_rule(first_switch, coil)
        self.assertEqual({19}, driver.switch_addresses)
        self.assertIsNotNone(driver.switch_rule)
        self.assertEqual(
            [packet(Command.SOLENOID_INPUT, 18, driver.address | 0x80)],
            [message for message in self.serial.messages
             if message[1] == Command.SOLENOID_INPUT])
        self.assertFalse(any(message[1] == Command.KICK_SOLENOIDS
                             for message in self.serial.messages))

        self.serial.messages.clear()
        platform.clear_hw_rule(second_switch, coil)
        self.assertFalse(driver.switch_addresses)
        self.assertIsNone(driver.switch_rule)
        self.assertEqual(
            [packet(Command.SOLENOID_INPUT, 19, driver.address | 0x80)],
            [message for message in self.serial.messages
             if message[1] == Command.SOLENOID_INPUT])
        self.assertTrue(any(message[1] == Command.CONFIGURE_SOLENOID
                            for message in self.serial.messages))
        self.assertTrue(any(message[1] == Command.KICK_SOLENOIDS
                            for message in self.serial.messages))

    def test_single_wound_iflip_flipper_without_eos(self):
        flipper = self.machine.flippers["f_single_iflip"]
        self.serial.messages.clear()

        flipper.enable()

        configs = [message for message in self.serial.messages
                   if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 12]
        associations = [message for message in self.serial.messages
                        if message[1] == Command.SOLENOID_INPUT]
        self.assertEqual(1, len(configs))
        self.assertTrue(configs[0][3] & 0x08)
        self.assertEqual(128, configs[0][6])
        self.assertIn(bytes((18, 12)), [message[2:4] for message in associations])
        self.assertFalse(any(message[1] == Command.SET_IFLIP_INPUT
                             for message in self.serial.messages))

        flipper.disable()
        self.assertIn(bytes((18, 0x8c)), [message[2:4] for message in self.serial.messages
                                         if message[1] == Command.SOLENOID_INPUT])

    def test_single_wound_iflip_flipper_with_eos_without_repulse(self):
        flipper = self.machine.flippers["f_single_iflip_eos"]
        self.serial.messages.clear()

        flipper.enable()

        eos = next(message for message in self.serial.messages
                   if message[1] == Command.SET_IFLIP_INPUT)
        self.assertEqual(bytes((0x8d, 41, 19, 255)), eos[2:6])
        config = next(message for message in self.serial.messages
                      if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 13)
        self.assertEqual(2, config[3] & 0x07)
        self.assertEqual(102, config[6])

        flipper.disable()
        eos = [message for message in self.serial.messages
               if message[1] == Command.SET_IFLIP_INPUT][-1]
        self.assertEqual(bytes((13, 41, 255, 255)), eos[2:6])
        config = [message for message in self.serial.messages
                  if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 13][-1]
        self.assertEqual(2, config[3] & 0x07)

    def test_dual_wound_iflip_flipper_and_hold_overwrite(self):
        flipper = self.machine.flippers["f_dual_iflip"]
        self.serial.messages.clear()

        flipper.enable()

        associations = [message[2:4] for message in self.serial.messages
                        if message[1] == Command.SOLENOID_INPUT]
        self.assertIn(bytes((18, 14)), associations)
        self.assertIn(bytes((18, 9)), associations)
        main = next(message for message in self.serial.messages
                    if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 14)
        hold = next(message for message in self.serial.messages
                    if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 9)
        self.assertEqual(153, main[6])
        self.assertEqual(12, hold[5])
        self.assertEqual(95, hold[6])
        self.assertEqual(32, hold[4])
        self.assertFalse(any(message[1] == Command.SET_IFLIP_INPUT
                             for message in self.serial.messages))

        flipper.disable()
        removals = [message[2:4] for message in self.serial.messages
                    if message[1] == Command.SOLENOID_INPUT and message[3] & 0x80]
        self.assertIn(bytes((18, 0x8e)), removals)
        self.assertIn(bytes((18, 0x89)), removals)

    def test_dual_wound_iflip_eos_repulse_and_recycle_restore(self):
        flipper = self.machine.flippers["f_dual_iflip_eos"]
        self.serial.messages.clear()

        flipper.enable()

        associations = [message[2:4] for message in self.serial.messages
                        if message[1] == Command.SOLENOID_INPUT]
        self.assertIn(bytes((18, 15)), associations)
        self.assertIn(bytes((18, 8)), associations)
        eos = next(message for message in self.serial.messages
                   if message[1] == Command.SET_IFLIP_INPUT)
        self.assertEqual(bytes((0xcf, 43, 19, 255)), eos[2:6])
        main = next(message for message in self.serial.messages
                    if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 15)
        self.assertEqual(3, main[3] & 0x07)
        self.assertEqual(128, main[6])

        flipper.disable()
        eos = [message for message in self.serial.messages
               if message[1] == Command.SET_IFLIP_INPUT][-1]
        self.assertEqual(bytes((15, 43, 255, 255)), eos[2:6])
        main = [message for message in self.serial.messages
                if message[1] == Command.CONFIGURE_SOLENOID and message[2] == 15][-1]
        self.assertEqual(2, main[3] & 0x07)

    def test_eos_repulse_recycle_conversion_boundaries(self):
        platform = self.machine.hardware_platforms["copperhead"]
        driver = self.machine.coils["c_iflip"].hw_driver
        enable_switch = SimpleNamespace(
            hw_switch=self.machine.switches["s_copperhead"].hw_switch)
        eos_switch = SimpleNamespace(
            hw_switch=self.machine.switches["s_copperhead_eos"].hw_switch)
        original_recycle = driver.platform_settings["recycle"]
        cases = ((32, 1, 0, 1), (32, 64, 0, 2), (32, 65, 0, 3),
                 (32, 32, 4, 4), (32, 500, 0, 7))
        try:
            for pulse_ms, debounce_ms, base_recycle, expected in cases:
                with self.subTest(pulse_ms=pulse_ms, debounce_ms=debounce_ms,
                                  base_recycle=base_recycle):
                    driver.platform_settings["recycle"] = base_recycle
                    coil = SimpleNamespace(
                        hw_driver=driver,
                        pulse_settings=SimpleNamespace(duration=pulse_ms, power=0.5),
                        hold_settings=SimpleNamespace(power=0.25))
                    repulse = SimpleNamespace(enable_repulse=True, debounce_ms=debounce_ms)
                    self.serial.messages.clear()

                    platform.set_pulse_on_hit_and_enable_and_release_and_disable_rule(
                        enable_switch, eos_switch, coil, repulse)
                    configured = next(
                        message for message in self.serial.messages
                        if message[1] == Command.CONFIGURE_SOLENOID)
                    self.assertEqual(expected, configured[3] & 0x07)

                    platform.clear_hw_rule(enable_switch, coil)
                    restored = [message for message in self.serial.messages
                                if message[1] == Command.CONFIGURE_SOLENOID][-1]
                    self.assertEqual(base_recycle, restored[3] & 0x07)
        finally:
            driver.platform_settings["recycle"] = original_recycle

    def test_dc_motor_reverse_power_and_stop(self):
        platform = self.machine.hardware_platforms["copperhead"]
        motor = self.machine.dc_motors["motor_test"].hw_motor
        self.serial.messages.clear()

        motor.reverse_pulse(.02, .25)
        config = next(message for message in self.serial.messages
                      if message[1] == Command.CONFIGURE_SOLENOID)
        self.assertEqual(55, config[2])
        self.assertEqual(bytes((0x22, 32, 32, 32)), config[3:7])
        self.assertIn(55, platform._spi_commands[20])
        motor.stop()
        self.assertNotIn(54, platform._spi_commands[20])
        self.assertNotIn(55, platform._spi_commands[20])

    def test_status_is_polled_once_per_second(self):
        initial = sum(message[1] == Command.GET_STATUS for message in self.serial.messages)
        self.advance_time_and_run(1.1)
        final = sum(message[1] == Command.GET_STATUS for message in self.serial.messages)
        self.assertGreater(final, initial)

    def test_fragmented_and_delayed_runtime_responses(self):
        platform = self.machine.hardware_platforms["copperhead"]
        connection = platform.connections[20]
        self.serial.response_chunks = [1, 2, 3]
        self.serial.messages.clear()
        self.advance_time_and_run(.11)
        self.assertFalse(self.machine.stop_future.done())
        self.assertTrue(any(message[1] == Command.READ_MATRIX_INPUTS
                            for message in self.serial.messages))

        self.serial.delay_command = Command.GET_STATUS
        transaction = self.machine.clock.loop.create_task(connection.transact(Command.GET_STATUS))
        self.advance_time_and_run(.1)
        self.assertFalse(transaction.done())
        self.serial.release_delayed_responses()
        self.advance_time_and_run(.01)
        self.assertEqual(bytes((0, 0)), transaction.result())
        self.serial.delay_command = None

    def test_runtime_status_fault_stops_mpf(self):
        self.serial.status_result = (1, 5)
        self.advance_time_and_run(1.1)
        self.assertTrue(self.machine.stop_future.done())
        self.assertIn("fault 5: control-loop overrun", self.machine.stop_future.result())

    def test_runtime_crc_failure_stops_mpf(self):
        self.serial.corrupt_command = Command.READ_SPI_INPUTS
        self.advance_time_and_run(.2)
        self.assertTrue(self.machine.stop_future.done())
        self.assertIn("communication failure", self.machine.stop_future.result())
        self.assertIn("CRC-8", self.machine.stop_future.result())

    def test_runtime_timeout_and_partial_response_stop_mpf(self):
        self.serial.partial_command = Command.READ_INPUTS
        self.serial.partial_response_bytes = 2
        self.advance_time_and_run(.7)
        self.assertTrue(self.machine.stop_future.done())
        self.assertIn("0x08", self.machine.stop_future.result())

    def test_runtime_write_failure_stops_mpf(self):
        self.serial.fail_write_command = Command.READ_INPUTS
        self.advance_time_and_run(.2)
        self.assertTrue(self.machine.stop_future.done())
        self.assertIn("injected Copperhead USB write failure", self.machine.stop_future.result())

    def test_runtime_partial_response_disconnect_stops_mpf(self):
        self.serial.disconnect_command = Command.READ_MATRIX_INPUTS
        self.serial.partial_response_bytes = 3
        self.advance_time_and_run(.2)
        self.assertTrue(self.machine.stop_future.done())
        reason = self.machine.stop_future.result()
        self.assertIn("connection failed during command 0x19", reason)
        self.assertIn("injected Copperhead USB disconnect", reason)

    def test_final_matrix_light_successor_is_validated_when_used(self):
        platform = self.machine.hardware_platforms["copperhead"]
        final_light = platform.configure_light("20-63", "matrix", None, None)
        self.assertEqual("20-64", final_light.get_successor_number())
        with self.assertRaisesRegex(AssertionError, "outside 0-63"):
            platform.configure_light(final_light.get_successor_number(), "matrix", None, None)

    def test_neopixel_length_discovery_and_boundaries(self):
        platform = self.machine.hardware_platforms["copperhead"]

        # Length is independent of the two startup-glow bits. A zero length
        # nibble is the firmware encoding for a full 256-pixel output.
        self.assertEqual([16, 64, 256], platform._decode_neopixel_lengths(
            [0x51, 0x64, 0x40, 0x80, 0xff]))
        self.assertEqual(
            [16, 16, 16, 32, 64, 80, 256, 256],
            [platform._neopixel_length_for_required_pixels(value)
             for value in (0, 1, 16, 17, 64, 65, 241, 256)])

        channels = platform.parse_light_number_to_channels("20-2-255", None)
        self.assertEqual(["20-2-765", "20-2-766", "20-2-767"],
                         [item["number"] for item in channels])
        with self.assertRaisesRegex(AssertionError, "pixel 256 is out of range"):
            platform.parse_light_number_to_channels("20-2-256", None)

        final_light = platform.configure_light("20-2-767", None, None, None)
        self.assertEqual("20-2-768", final_light.get_successor_number())
        with self.assertRaisesRegex(AssertionError, "channel 768 is out of range"):
            platform.configure_light(final_light.get_successor_number(), None, None, None)

        self.assertEqual(768, platform.configure_light("20-1-0", None, None, None).address)
        self.assertEqual(1536, platform.configure_light("20-2-0", None, None, None).address)

    def test_shutdown_clears_all_outputs(self):
        platform = self.machine.hardware_platforms["copperhead"]
        rule_driver = platform.drivers["20-8"]
        rule_driver.switch_addresses.update((18, 19))
        rule_driver.switch_rule = object()
        self.serial.messages.clear()
        platform.stop()
        # Simulate delayed MPF light callbacks racing with asynchronous shutdown.
        connection = platform.connections[20]
        platform.queue_interpolation(connection, 0, 255, 0)
        platform.queue_interpolation(connection, 0x2000, 255, 0)
        platform.light_sync()
        self.advance_time_and_run(.1)

        self.assertIn(packet(Command.KICK_SOLENOIDS, 0, 0, 0), self.serial.messages)
        self.assertIn(packet(Command.KICK_SPI_SOLENOIDS, *bytes(13)), self.serial.messages)
        clears = [message for message in self.serial.messages if message[1] == Command.INTERPOLATE]
        self.assertTrue(any(message[2:4] == bytes((0, 0)) and
                            int.from_bytes(message[4:6], "big") == 2304 for message in clears))
        self.assertTrue(any(message[2:4] == bytes((0x20, 0)) and
                            int.from_bytes(message[4:6], "big") == 64 for message in clears))
        self.assertTrue(any(message[1] == Command.GET_STATUS for message in self.serial.messages))
        status_index = max(index for index, message in enumerate(self.serial.messages)
                           if message[1] == Command.GET_STATUS)
        gpio_clear = packet(Command.KICK_SOLENOIDS, 0, 0, 0)
        spi_clear = packet(Command.KICK_SPI_SOLENOIDS, *bytes(13))
        rule_clears = [
            packet(Command.SOLENOID_INPUT, 18, 8 | 0x80),
            packet(Command.SOLENOID_INPUT, 19, 8 | 0x80),
        ]
        stepper_clear_index = next(index for index, message in enumerate(self.serial.messages)
                                   if message[1] == Command.STEPPER_COMMAND and not message[3] & 1)
        servo_clear_index = next(index for index, message in enumerate(self.serial.messages)
                                 if message[1] == Command.INTERPOLATE and
                                 message[2:4] == bytes((0x30, 17)) and message[8] == 0)
        lamp_clear_index = next(index for index, message in enumerate(self.serial.messages)
                                if message[1] == Command.INTERPOLATE and
                                message[2:4] == bytes((0x20, 0)))
        pixel_clear_index = next(index for index, message in enumerate(self.serial.messages)
                                 if message[1] == Command.INTERPOLATE and
                                 message[2:4] == bytes((0, 0)))
        ordered_indices = [
            *(self.serial.messages.index(message) for message in rule_clears),
            self.serial.messages.index(gpio_clear),
            self.serial.messages.index(spi_clear),
            stepper_clear_index,
            servo_clear_index,
            lamp_clear_index,
            pixel_clear_index,
            status_index,
        ]
        self.assertEqual(sorted(ordered_indices), ordered_indices)
        self.assertTrue(all(self.serial.messages.index(message) < status_index for message in clears))
        self.assertFalse(rule_driver.switch_addresses)
        self.assertIsNone(rule_driver.switch_rule)
        self.assertNotIn(interpolation_packet(0, 0, b"\xff"), self.serial.messages)
        self.assertNotIn(interpolation_packet(0x2000, 0, b"\xff"), self.serial.messages)
        self.assertFalse(platform.serial_connections)

    def test_shutdown_timeout_and_bad_crc_are_bounded(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.serial.drop_command = Command.GET_STATUS
        platform.stop()
        self.advance_time_and_run(.91)
        self.assertTrue(platform._shutdown_task.done())
        self.assertFalse(platform.serial_connections)

    def test_shutdown_bad_crc_closes_connection(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.serial.corrupt_command = Command.GET_STATUS
        platform.stop()
        self.advance_time_and_run(.2)
        self.assertTrue(platform._shutdown_task.done())
        self.assertFalse(platform.serial_connections)

    def test_shutdown_consumes_inflight_input_response_before_status_barrier(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.serial.delay_command = Command.READ_INPUTS
        self.advance_time_and_run(.11)
        self.assertTrue(self.serial.delayed_responses)
        poll_tasks = list(platform._poll_tasks)

        platform.stop()
        self.advance_time_and_run(.1)
        self.assertFalse(platform._shutdown_task.done())

        self.serial.release_delayed_responses()
        self.advance_time_and_run(.1)

        self.assertTrue(platform._shutdown_task.done())
        self.assertFalse(platform.serial_connections)
        self.assertTrue(all(task.done() and not task.cancelled() for task in poll_tasks))
        self.assertEqual(Command.GET_STATUS, self.serial.messages[-1][1])

    def test_spi_ownership_and_reserved_matrix_switch(self):
        platform = self.machine.hardware_platforms["copperhead"]
        with self.assertRaisesRegex(AssertionError, "not a MULTI switch input"):
            platform.configure_switch("20-0", None, None)
        with self.assertRaisesRegex(AssertionError, "GPIO solenoid.*unavailable"):
            platform.configure_driver(None, "20-0", {})
        with self.assertRaisesRegex(AssertionError, "not owned by an 8C16S or 16S"):
            platform.configure_switch("20-64", None, None)
        with self.assertRaisesRegex(AssertionError, "not owned by an 8C16S"):
            platform.configure_driver(None, "20-56", {})
        switch = SimpleNamespace(hw_switch=SimpleNamespace(address=255))
        coil = SimpleNamespace(hw_driver=SimpleNamespace())
        with self.assertRaisesRegex(AssertionError, "switch 255"):
            platform._set_rule(switch, coil, True, False, False)

        with self.assertRaisesRegex(AssertionError, "reserved by a configured DC motor"):
            platform.configure_driver(None, "20-54", {})


class TestCopperheadStartupErrors(MpfTestCase):
    """Verify startup errors are bounded and identify the failed transaction."""

    def get_machine_path(self):
        return "tests/machine_files/copperhead/"

    def get_config_file(self):
        return "config.yaml"

    def get_platform(self):
        return False

    def _mock_loop(self):
        self.clock.mock_serial("com1", self.serial)

    def setUp(self):
        self.serial = MockCopperheadSerial()
        if self._testMethodName == "test_board_number_timeout":
            self.serial.drop_command = Command.GET_BOARD_NUMBER
        elif self._testMethodName == "test_bad_board_number_crc":
            self.serial.corrupt_command = Command.GET_BOARD_NUMBER
        elif self._testMethodName == "test_faulted_board":
            self.serial.status_result = (1, 5)
        else:
            self.serial.spi_results[0x25] = (4, 5, 0, 0)
        super().setUp()

    @expect_startup_error()
    def test_board_number_timeout(self):
        self.assertIsInstance(self.startup_error, TimeoutError)
        self.assertIn("0x00", str(self.startup_error))
        self.assertIn("com1", str(self.startup_error))

    @expect_startup_error()
    def test_bad_board_number_crc(self):
        self.assertIsInstance(self.startup_error, ValueError)
        self.assertIn("CRC-8", str(self.startup_error))

    @expect_startup_error()
    def test_spi_assignment_rejected(self):
        self.assertIsInstance(self.startup_error, AssertionError)
        self.assertIn("status 4 (overlap)", str(self.startup_error))

    @expect_startup_error()
    def test_faulted_board(self):
        self.assertIsInstance(self.startup_error, AssertionError)
        self.assertIn("status 1, fault 5", str(self.startup_error))


class TestCopperheadDuplicateBoards(MpfTestCase):
    """Reject duplicate board numbers found on separate serial ports."""

    def get_machine_path(self):
        return "tests/machine_files/copperhead/"

    def get_config_file(self):
        return "duplicate.yaml"

    def get_platform(self):
        return False

    def _mock_loop(self):
        self.clock.mock_serial("com1", self.serial1)
        self.clock.mock_serial("com2", self.serial2)

    def setUp(self):
        self.serial1 = MockCopperheadSerial()
        self.serial2 = MockCopperheadSerial()
        super().setUp()

    @expect_startup_error()
    def test_duplicate_board_numbers(self):
        self.assertIsInstance(self.startup_error, AssertionError)
        self.assertIn("Duplicate Copperhead board number 20", str(self.startup_error))


class TestCobraPinAutomaticOutputs(MpfTestCase):
    """Discover two CobraPin boards and initialize their onboard devices."""

    def get_machine_path(self):
        return "tests/machine_files/copperhead/"

    def get_config_file(self):
        return "cobrapin_auto.yaml"

    def get_platform(self):
        return False

    def _mock_loop(self):
        self.clock.mock_serial("com1", self.serial1)
        self.clock.mock_serial("com2", self.serial2)

    def setUp(self):
        self.serial1 = MockCopperheadSerial(42)
        self.serial2 = MockCopperheadSerial(43)
        for serial in (self.serial1, self.serial2):
            serial.hardware_model = 1
            serial.hardware_revision = (3, 0)
            serial.wings = [13, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 13]
            serial.multi_config_response = [0xff] * 48
            serial.multi_config_response[4:12] = [0] * 8
            serial.multi_config_response[12:20] = [0xc0, 0xc0, 0x40, 0x40, 0x40, 0x80, 0x80, 0x80]
            serial.spi_results.update({5: (0, 5, 0, 0), 6: (0, 6, 2, 31)})
        super().setUp()

    def test_cobrapin_profile_devices(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.assertEqual([13, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 13],
                         platform.connections[42].wings)
        self.assertEqual(4, self.machine.coils["c_profile_normal"].hw_driver.address)
        self.assertEqual(20, self.machine.coils["c_profile_iflip"].hw_driver.address)
        self.assertEqual(40, self.machine.coils["c_profile_iflip"].hw_driver.iflip_analog_input)
        self.assertEqual(4, self.machine.switches["s_profile"].hw_switch.address)

    def test_generated_outputs_and_activity_led(self):
        self.assertEqual(6, len(self.machine.digital_outputs))
        for board, serial in ((42, self.serial1), (43, self.serial2)):
            self.assertIn(packet(Command.CONFIGURE_SOLENOID, 0xfe, 29, 0, 0, 0), serial.messages)
            for suffix, address in (("power_enable", 28), ("relay_30", 30), ("relay_31", 31)):
                device = self.machine.digital_outputs["cobrapin_{}_{}".format(board, suffix)]
                self.assertEqual(address, device.hw_driver.address)
                self.assertEqual(board, device.hw_driver.connection.board_number)
                self.assertEqual(1.0, device.hw_driver.config.default_hold_power)
            kicks = [msg for msg in serial.messages if msg[1] == Command.KICK_SPI_SOLENOIDS]
            self.assertTrue(all(not any(msg[2:-1]) for msg in kicks))

    def test_user_events_enable_full_power_and_disable(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.post_event("relay_on")
        self.assertEqual({30}, platform._spi_commands[42])
        self.assertEqual(set(), platform._spi_commands[43])
        self.assertIn(packet(Command.CONFIGURE_SOLENOID, 30, 0x20, 127, 0, 127),
                      self.serial1.messages)
        self.post_event("relay_off")
        self.assertEqual(set(), platform._spi_commands[42])

    def test_reserved_outputs_reject_duplicate_owners(self):
        platform = self.machine.hardware_platforms["copperhead"]
        for address in (28, 29, 30, 31):
            with self.assertRaisesRegex(AssertionError, "reserved"):
                platform.configure_driver(SimpleNamespace(name="other"), "42-{}".format(address), {})

    def test_shutdown_clears_auxiliary_outputs(self):
        platform = self.machine.hardware_platforms["copperhead"]
        for suffix in ("power_enable", "relay_30", "relay_31"):
            self.machine.digital_outputs["cobrapin_42_" + suffix].enable()
        self.assertEqual({28, 30, 31}, platform._spi_commands[42])
        platform.stop()
        self.advance_time_and_run(.1)
        kicks = [msg for msg in self.serial1.messages if msg[1] == Command.KICK_SPI_SOLENOIDS]
        self.assertFalse(any(kicks[-1][2:-1]))


class TestCopperheadDiscoveryOnly(MpfTestCase):
    """Use the same partial initialization as mpf hardware scan."""

    get_machine_path = TestCobraPinAutomaticOutputs.get_machine_path
    get_config_file = TestCobraPinAutomaticOutputs.get_config_file
    get_platform = TestCobraPinAutomaticOutputs.get_platform
    _mock_loop = TestCobraPinAutomaticOutputs._mock_loop

    def setUp(self):
        self.serial1 = MockCopperheadSerial(42)
        self.serial2 = MockCopperheadSerial(43)
        for serial in (self.serial1, self.serial2):
            serial.hardware_model = 1
            serial.hardware_revision = (3, 0)
            serial.wings = [13, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 13]
        if self._testMethodName == "test_faulted_board_can_be_inspected":
            self.serial1.status_result = (1, 5)
        super().setUp()

    def _initialize_machine(self):
        task = asyncio.ensure_future(self.machine.initialize_core_and_hardware())
        self._wait_for_start(task, 10)

    def test_discovery_does_not_configure_or_clear_outputs(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.assertFalse(platform._runtime_setup_started)
        self.assertEqual({}, platform.spi_assignments)
        self.assertEqual([], platform._poll_tasks)
        self.assertNotIn("cobrapin_42_power_enable", self.machine.config["digital_outputs"])
        self.assertEqual({"enable_events": "relay_on", "disable_events": "relay_off"},
                         self.machine.config["digital_outputs"]["cobrapin_42_relay_30"])
        allowed = {Command.GET_BOARD_NUMBER, Command.GET_VERSION, Command.GET_WING_CONFIG,
                   Command.GET_MULTI_CONFIG, Command.GET_STATUS}
        for serial in (self.serial1, self.serial2):
            self.assertTrue(all(message[1] in allowed for message in serial.messages))
        message_counts = [len(self.serial1.messages), len(self.serial2.messages)]
        platform.stop()
        self.assertFalse(platform.serial_connections)
        self.assertIsNone(platform._shutdown_task)
        self.assertEqual(message_counts, [len(self.serial1.messages), len(self.serial2.messages)])

    def test_faulted_board_can_be_inspected(self):
        platform = self.machine.hardware_platforms["copperhead"]
        self.assertEqual(5, platform.connections[42].fault)
        self.assertIn("Fault: active (5)", platform.get_info_string())
        self.assertFalse(platform._runtime_setup_started)
