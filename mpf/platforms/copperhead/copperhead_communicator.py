"""Serial connection for one Copperhead controller."""

import asyncio

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, SerialException

from mpf.platforms.copperhead.copperhead_protocol import Command, RESPONSE_SIZES, packet, validate_response


class CopperheadCommunicator:
    """Own one USB CDC connection and its request/response serialization."""

    __slots__ = ["platform", "machine", "port", "baud", "reader", "writer", "lock",
                 "board_number", "firmware_version", "wings", "multi_config", "neopixel_lengths",
                 "status", "fault", "hardware_model", "hardware_revision"]

    def __init__(self, platform, port, baud):
        self.platform = platform
        self.machine = platform.machine
        self.port = port
        self.baud = baud
        self.reader = None
        self.writer = None
        self.lock = asyncio.Lock()
        self.board_number = None
        self.firmware_version = None
        self.hardware_model = 0
        self.hardware_revision = (0, 0)
        self.wings = []
        self.multi_config = []
        self.neopixel_lengths = []
        self.status = 0
        self.fault = 0

    async def connect(self):
        """Open, identify, and inspect this controller."""
        self.platform.log.info("Connecting to %s at %sbps", self.port, self.baud)
        self.platform.debug_log("Opening Copperhead serial port %s at %s baud", self.port, self.baud)
        try:
            connector = self.machine.clock.open_serial_connection(
                url=self.port, baudrate=self.baud, limit=0, xonxoff=False,
                bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE)
            self.reader, self.writer = await connector
        except SerialException:
            raise

        serial = self.writer.transport.serial
        serial.reset_input_buffer()
        self.reader._buffer = bytearray()  # pylint: disable=protected-access

        self.platform.debug_log("Identifying Copperhead controller on %s", self.port)
        board = await self.transact(Command.GET_BOARD_NUMBER)
        self.board_number = int.from_bytes(board, "big")
        self.platform.register_connection(self)
        version = await self.transact(Command.GET_VERSION)
        self.firmware_version = tuple(version[:4])
        self.hardware_model = int.from_bytes(version[4:6], "big")
        self.hardware_revision = tuple(version[6:8])
        if self.firmware_version < (0, 0, 0, 13):
            raise AssertionError("Copperhead firmware 0.0.0.13 or newer is required")
        self.wings = list(await self.transact(Command.GET_WING_CONFIG))
        self.multi_config = list(await self.transact(Command.GET_MULTI_CONFIG))
        self.neopixel_lengths = self.platform._decode_neopixel_lengths(self.multi_config)
        status = await self.transact(Command.GET_STATUS)
        self.status, self.fault = status
        self.platform.debug_log("Discovered Copperhead board %s on %s: status=%s fault=%s wings=%s",
                                self.board_number, self.port, self.status, self.fault, self.wings)
        self.platform.log.info(
            "Discovered Copperhead board %s on %s: firmware=%s status=%s fault=%s wings=%s",
            self.board_number, self.port,
            ".".join(str(value) for value in self.firmware_version),
            self.status, self.fault, self.wings)

    async def transact(self, command, *payload):
        """Send a command and read its fixed-length response."""
        async with self.lock:
            if not self.writer:
                raise asyncio.CancelledError
            request = packet(command, *payload)
            self.platform.debug_log("Copperhead %s send: %s", self.port, request.hex(" "))
            try:
                self.writer.write(request)
                if hasattr(self.writer, "drain"):
                    await self.writer.drain()
                response = await asyncio.wait_for(
                    self.reader.readexactly(RESPONSE_SIZES[Command(command)]),
                    self.platform.config["response_timeout_ms"] / 1000.0)
            except asyncio.TimeoutError as exc:
                raise TimeoutError(
                    "Copperhead {} timed out waiting for command 0x{:02x} response on {}".format(
                        self.board_number if self.board_number is not None else "unidentified",
                        int(command), self.port)) from exc
            except asyncio.IncompleteReadError as exc:
                raise ConnectionError(
                    "Copperhead connection closed during command 0x{:02x} response on {}; "
                    "received {} of {} bytes".format(
                        int(command), self.port, len(exc.partial), exc.expected)) from exc
            except OSError as exc:
                raise ConnectionError(
                    "Copperhead {} connection failed during command 0x{:02x} on {}: {}".format(
                        self.board_number if self.board_number is not None else "unidentified",
                        int(command), self.port, exc)) from exc
            try:
                data = validate_response(response, command)
            except ValueError as exc:
                raise ValueError("Invalid Copperhead command 0x{:02x} response on {}: {}".format(
                    int(command), self.port, exc)) from exc
            self.platform.debug_log("Copperhead %s receive: %s", self.port, response.hex(" "))
            return data

    def send(self, message):
        """Send a command which has no response."""
        if not self.writer:
            return
        self.platform.debug_log("Copperhead %s send: %s", self.port, message.hex(" "))
        self.writer.write(message)

    async def send_and_confirm(self, messages):
        """Send ordered shutdown messages and confirm firmware consumed them."""
        async with self.lock:
            if not self.writer:
                raise ConnectionError("Copperhead connection is already closed")
            for item in messages:
                if isinstance(item, tuple):
                    step, message = item
                else:
                    step, message = "output clear", item
                self.platform.debug_log("Copperhead board %s shutdown step: %s; send: %s",
                                        self.board_number, step, message.hex(" "))
                self.writer.write(message)
            self.platform.debug_log("Copperhead board %s shutdown step: drain output commands",
                                    self.board_number)
            if hasattr(self.writer, "drain"):
                await self.writer.drain()

            request = packet(Command.GET_STATUS)
            self.platform.debug_log("Copperhead board %s shutdown step: send status barrier; send: %s",
                                    self.board_number, request.hex(" "))
            self.writer.write(request)
            if hasattr(self.writer, "drain"):
                await self.writer.drain()
            response = await self.reader.readexactly(RESPONSE_SIZES[Command.GET_STATUS])
            status = validate_response(response, Command.GET_STATUS)
            self.status, self.fault = status
            self.platform.debug_log(
                "Copperhead board %s shutdown step: status barrier response status=%s fault=%s",
                self.board_number, self.status, self.fault)

    def stop(self):
        """Close this serial connection."""
        if self.writer:
            self.writer.close()
            self.writer = None

    def __repr__(self):
        return "Copperhead {} on {}".format(self.board_number, self.port)
