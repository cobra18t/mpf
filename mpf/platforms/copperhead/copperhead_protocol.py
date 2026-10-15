"""Copperhead USB protocol helpers."""

from enum import IntEnum


PREFIX = 0x20


class Command(IntEnum):

    """Commands used by the first MPF integration milestone."""

    GET_BOARD_NUMBER = 0x00
    GET_VERSION = 0x02
    SET_IFLIP_INPUT = 0x0A
    KICK_SOLENOIDS = 0x07
    READ_INPUTS = 0x08
    GET_MULTI_CONFIG = 0x09
    GET_WING_CONFIG = 0x0D
    CONFIGURE_SOLENOID = 0x14
    CONFIGURE_INPUT = 0x15
    SOLENOID_INPUT = 0x17
    READ_MATRIX_INPUTS = 0x19
    READ_SPI_INPUTS = 0x1A
    GET_STATUS = 0x1B
    STEPPER_COMMAND = 0x1C
    KICK_SPI_SOLENOIDS = 0x1F
    SET_SPI_BOARD_FUNCTION = 0x20
    GET_STEPPER_POSITION = 0x22
    INTERPOLATE = 0x40


RESPONSE_SIZES = {
    Command.GET_BOARD_NUMBER: 7,
    Command.GET_VERSION: 11,
    Command.READ_INPUTS: 9,
    Command.GET_MULTI_CONFIG: 51,
    Command.GET_WING_CONFIG: 15,
    Command.READ_MATRIX_INPUTS: 11,
    Command.READ_SPI_INPUTS: 21,
    Command.GET_STATUS: 5,
    Command.SET_SPI_BOARD_FUNCTION: 7,
    Command.GET_STEPPER_POSITION: 8,
}


def crc8(data):
    """Return the OPP-compatible CRC-8 for data."""
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def packet(command, *payload):
    """Build a Copperhead command packet."""
    body = bytes((PREFIX, int(command), *payload))
    return body + bytes((crc8(body),))


def interpolation_packet(offset, duration_ms, data):
    """Build a variable-length Copperhead interpolation packet."""
    data = bytes(data)
    if not 0 <= offset <= 0xffff:
        raise ValueError("Interpolation offset must fit in 16 bits")
    if not 0 <= duration_ms <= 0xffff:
        raise ValueError("Interpolation duration must fit in 16 bits")
    if len(data) > 0xffff:
        raise ValueError("Interpolation payload must fit in 16 bits")
    message = bytearray((PREFIX, Command.INTERPOLATE))
    message.extend(offset.to_bytes(2, "big"))
    message.extend(len(data).to_bytes(2, "big"))
    message.extend(duration_ms.to_bytes(2, "big"))
    message.extend(data)
    message.append(crc8(message))
    return bytes(message)


def validate_response(response, command):
    """Validate and return the payload of a fixed-length response."""
    expected = RESPONSE_SIZES[Command(command)]
    if len(response) != expected:
        raise ValueError("Expected {} bytes for command 0x{:02x}, received {}".format(
            expected, int(command), len(response)))
    if response[:2] != bytes((PREFIX, int(command))):
        raise ValueError("Copperhead response prefix or command does not match")
    if response[-1] != crc8(response[:-1]):
        raise ValueError("Copperhead response has an invalid CRC-8")
    return response[2:-1]


def bitmap_packet(command, addresses, first, count):
    """Build an authoritative little-endian solenoid bitmap command."""
    value = 0
    for address in addresses:
        if not first <= address < first + count:
            raise ValueError("Solenoid {} is outside {}-{}".format(address, first, first + count - 1))
        value |= 1 << (address - first)
    return packet(command, *value.to_bytes((count + 7) // 8, "little"))


def unpack_bits(payload, count):
    """Unpack a little-endian bit array."""
    return [bool(payload[index >> 3] & (1 << (index & 7))) for index in range(count)]
