"""Tests for Copperhead packet encoding independent of serial hardware."""

import asyncio
import unittest
from unittest.mock import AsyncMock
from types import SimpleNamespace

from mpf.platforms.copperhead.copperhead_protocol import (
    Command, bitmap_packet, crc8, interpolation_packet, packet, unpack_bits, validate_response,
)
from mpf.platforms.copperhead.copperhead import CopperheadHardwarePlatform
from mpf.platforms.copperhead.copperhead_servo import CopperheadServo


class TestCopperheadProtocol(unittest.TestCase):
    """Verify the protocol primitives used by the MPF platform."""

    def test_known_commands(self):
        self.assertEqual(bytes.fromhex("20 00 79"), packet(Command.GET_BOARD_NUMBER))
        self.assertEqual(bytes.fromhex("20 08 41"), packet(Command.READ_INPUTS))
        self.assertEqual(bytes.fromhex("20 09 46"), packet(Command.GET_MULTI_CONFIG))
        self.assertEqual(bytes.fromhex("20 0d 5a"), packet(Command.GET_WING_CONFIG))

    def test_cobrapin_auto_spi_preset(self):
        connection = SimpleNamespace(hardware_model=1, hardware_revision=(3, 0),
                                     transact=AsyncMock(side_effect=[
                                         bytes((0, 0, 31, 31)), bytes((0, 5, 0, 0)),
                                         bytes((0, 6, 2, 31)), bytes((0, 5, 1, 4))]))
        platform = SimpleNamespace(
            connections={42: connection}, config={"spi_functions": [
                {"board": 42, "function": "8c16s", "allocation": "next_spi0",
                 "start_a": 31, "start_b": 31}]}, spi_assignments={},
            SPI_FUNCTIONS=CopperheadHardwarePlatform.SPI_FUNCTIONS,
            SPI_ALLOCATIONS=CopperheadHardwarePlatform.SPI_ALLOCATIONS,
            log=SimpleNamespace(info=lambda *args: None),
            _require_spi_success=lambda *args: None)
        asyncio.run(CopperheadHardwarePlatform._configure_spi_functions(platform))
        self.assertEqual([
            ((Command.SET_SPI_BOARD_FUNCTION, 0, 31, 31),),
            ((Command.SET_SPI_BOARD_FUNCTION, 5, 0, 0),),
            ((Command.SET_SPI_BOARD_FUNCTION, 6, 2, 31),),
            ((Command.SET_SPI_BOARD_FUNCTION, 0x25, 31, 31),),
        ], [call.args and (call.args,) for call in connection.transact.call_args_list])
        self.assertEqual(3, len(platform.spi_assignments[42]))

        # No user SPI entries: both onboard functions still initialize.
        connection.transact.reset_mock()
        connection.transact.side_effect = [bytes((0, 0, 31, 31)),
                                           bytes((0, 5, 0, 0)), bytes((0, 6, 2, 31))]
        platform.config["spi_functions"] = None
        asyncio.run(CopperheadHardwarePlatform._configure_spi_functions(platform))
        self.assertEqual(3, connection.transact.call_count)
        self.assertEqual(["8c16s", "16s"],
                         [item["function"] for item in platform.spi_assignments[42]])

    def test_unknown_hardware_has_no_auto_preset(self):
        connection = SimpleNamespace(hardware_model=1, hardware_revision=(4, 0),
                                     transact=AsyncMock())
        platform = SimpleNamespace(connections={42: connection},
                                   config={"spi_functions": []}, spi_assignments={})
        asyncio.run(CopperheadHardwarePlatform._configure_spi_functions(platform))
        connection.transact.assert_not_called()

    def test_automatic_output_rejects_conflicting_hardware_fields(self):
        connection = SimpleNamespace(hardware_model=1, hardware_revision=(3, 0))
        def config_error(message, code):
            raise AssertionError(message)
        for key, value in (("number", "42-30"), ("platform", "virtual"), ("type", "light")):
            platform = SimpleNamespace(connections={42: connection},
                machine=SimpleNamespace(config={"digital_outputs": {
                    "cobrapin_42_power_enable": {key: value}}}),
                raise_config_error=config_error)
            with self.assertRaisesRegex(AssertionError, "requires " + key):
                CopperheadHardwarePlatform._configure_cobrapin_auxiliary_outputs(platform)

    def test_extended_version_response(self):
        payload = bytes((0, 0, 0, 13, 0x12, 0x34, 3, 0))
        response = packet(Command.GET_VERSION, *payload)
        self.assertEqual(11, len(response))
        self.assertEqual(payload, validate_response(response, Command.GET_VERSION))
        with self.assertRaises(ValueError):
            validate_response(packet(Command.GET_VERSION, 0, 0, 0, 12), Command.GET_VERSION)

    def test_interpolation_packet(self):
        message = interpolation_packet(0x3000, 1250, bytes((150,)))
        self.assertEqual(bytes.fromhex("20 40 30 00 00 01 04 e2 96"), message[:-1])
        self.assertEqual(message[-1], crc8(message[:-1]))

    def test_servo_speed_and_stop(self):
        class PlatformStub:
            def __init__(self):
                self.commands = []

            def send_interpolation(self, connection, address, data, duration_ms):
                self.commands.append((connection, address, bytes(data), duration_ms))

        platform = PlatformStub()
        connection = SimpleNamespace(board_number=20)
        servo = CopperheadServo(platform, connection, 17)
        servo.current_position = 150
        servo.set_speed_limit(0.1)
        servo.go_to_position(200 / 255)
        self.assertEqual((connection, 0x3011, bytes((200,)), 1961), platform.commands[-1])

        servo.set_speed_limit(0)
        servo.go_to_position(150 / 255)
        self.assertEqual(0, platform.commands[-1][3])
        servo.stop()
        self.assertEqual((connection, 0x3011, b"\x00", 0), platform.commands[-1])

    def test_gpio_solenoid_bitmap(self):
        self.assertEqual(bytes.fromhex("20 07 01 80 80 6b"),
                         bitmap_packet(Command.KICK_SOLENOIDS, (0, 15, 23), 0, 24))

    def test_spi_solenoid_bitmap(self):
        message = bitmap_packet(Command.KICK_SPI_SOLENOIDS, (24, 40, 127), 24, 104)
        self.assertEqual(16, len(message))
        self.assertEqual(1, message[2])
        self.assertEqual(1, message[4])
        self.assertEqual(0x80, message[14])

    def test_response_validation_and_bits(self):
        response = packet(Command.GET_STATUS, 0, 0)
        self.assertEqual(bytes((0, 0)), validate_response(response, Command.GET_STATUS))
        self.assertEqual([True, False, True, False], unpack_bits(bytes((5,)), 4))

        multi = bytes(range(48))
        self.assertEqual(multi, validate_response(
            packet(Command.GET_MULTI_CONFIG, *multi), Command.GET_MULTI_CONFIG))

        response = packet(Command.GET_STEPPER_POSITION, 2,
                          *(-123456).to_bytes(4, "big", signed=True))
        payload = validate_response(response, Command.GET_STEPPER_POSITION)
        self.assertEqual(2, payload[0])
        self.assertEqual(-123456, int.from_bytes(payload[1:], "big", signed=True))

    def test_invalid_crc_is_rejected(self):
        response = bytearray(packet(Command.GET_STATUS, 0, 0))
        response[-1] ^= 1
        with self.assertRaises(ValueError):
            validate_response(bytes(response), Command.GET_STATUS)

    def test_duplicate_board_number_is_rejected(self):
        class PlatformStub:
            connections = {}
            _gpio_commands = {}
            _spi_commands = {}

        class ConnectionStub:
            board_number = 20

            def __init__(self, port):
                self.port = port
                self.stopped = False

            def stop(self):
                self.stopped = True

        platform = PlatformStub()
        first = ConnectionStub("com1")
        second = ConnectionStub("com2")
        CopperheadHardwarePlatform.register_connection(platform, first)
        with self.assertRaisesRegex(AssertionError, "Duplicate Copperhead board number 20"):
            CopperheadHardwarePlatform.register_connection(platform, second)
        self.assertTrue(second.stopped)

    def test_iflip_and_high_speed_configuration_bits(self):
        class ConnectionStub:
            def __init__(self):
                self.messages = []

            def send(self, message):
                self.messages.append(message)

        connection = ConnectionStub()
        driver = SimpleNamespace(
            address=15,
            connection=connection,
            config=SimpleNamespace(default_pulse_ms=32, default_pulse_power=1.0,
                                   default_hold_power=0.25),
            platform_settings={"iflip": True, "high_speed": False, "recycle": 3})
        CopperheadHardwarePlatform.configure_solenoid(
            SimpleNamespace(_power=CopperheadHardwarePlatform._power), driver,
            None, None, auto_clear=True, can_cancel=False)
        self.assertEqual(0x8b, connection.messages[-1][3])
        self.assertEqual(64, connection.messages[-1][4])
        self.assertEqual(255, connection.messages[-1][6])

        driver.platform_settings = {"iflip": False, "high_speed": True}
        CopperheadHardwarePlatform.configure_solenoid(
            SimpleNamespace(_power=CopperheadHardwarePlatform._power), driver,
            None, None, auto_clear=True, can_cancel=False)
        self.assertEqual(0x90, connection.messages[-1][3])
        self.assertEqual(32, connection.messages[-1][4])
        self.assertEqual(127, connection.messages[-1][6])

    def test_delayed_solenoid_configuration(self):
        class ConnectionStub:
            def __init__(self):
                self.messages = []

            def send(self, message):
                self.messages.append(message)

        connection = ConnectionStub()
        driver = SimpleNamespace(
            address=8,
            connection=connection,
            config=SimpleNamespace(default_pulse_ms=32, default_pulse_power=1.0,
                                   default_hold_power=0),
            platform_settings={"iflip": False, "high_speed": False, "recycle": 7})
        CopperheadHardwarePlatform.configure_solenoid(
            SimpleNamespace(_power=CopperheadHardwarePlatform._power), driver,
            None, None, auto_clear=True, can_cancel=False, delay_ms=255)
        message = connection.messages[-1]
        self.assertEqual(0xc7, message[3])
        self.assertEqual(255, message[4])
        self.assertEqual(32, message[5])
        self.assertEqual(127, message[6])

        with self.assertRaisesRegex(AssertionError, "between 0 and 255ms"):
            CopperheadHardwarePlatform.configure_solenoid(
                SimpleNamespace(_power=CopperheadHardwarePlatform._power), driver,
                None, None, auto_clear=True, can_cancel=False, delay_ms=256)


if __name__ == "__main__":
    unittest.main()
