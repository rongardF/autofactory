# Copyright (c) 2026, Autofactory
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""SIMATIC RF240R driver for the RF200 ASCII protocol over RS-232.

Threads:

* receive thread: splits the serial stream into frames, routes asynchronous
  presence messages (ANW-MELD) to the presence queue and acknowledgements to
  the single waiting command;
* presence thread: handles tag arrival/departure (MDS-STATUS) so the receive
  thread is always free to deliver acknowledgements;
* heartbeat: a ROS timer polling SLG-STATUS mode 0 for line monitoring and
  presence re-sync.

All commands are serialised by ``_cmd_lock``.
"""

import queue
import threading
import time
from typing import Callable, Final

import serial
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.timer import Timer

from rfid_scanner.enums.ascii_command_enum import AsciiCommandEnum
from rfid_scanner.enums.reader_error_enum import ReaderErrorEnum
from rfid_scanner.exceptions.command_timeout_error import CommandTimeoutError
from rfid_scanner.exceptions.frame_error import FrameError
from rfid_scanner.exceptions.reader_fatal_error import ReaderFatalError
from rfid_scanner.exceptions.tag_access_error import TagAccessError
from rfid_scanner.interface.rfid_reader import RfidReader
from rfid_scanner.models.ascii_frame_dto import AsciiFrameDTO
from rfid_scanner.models.node_config_dto import NodeConfigDTO
from rfid_scanner.models.tag_info_dto import TagInfoDTO
from rfid_scanner.utils.ascii_frame_codec import FrameAssembler, decode_frame, encode_frame

RF240R_HW_TYPE: Final[int] = 0x33

# RESET: scan time 0, "with presence" + single tag, reset red LED, dili 0,
# mtag 0x0001, fcon 0, Ftim ISO general.
_RESET_PARAMS: Final[bytes] = bytes([0x00, 0x25, 0x02, 0x00, 0x00, 0x01, 0x00, 0x01])
_SLG_STATUS_MODE0: Final[bytes] = bytes([0x00, 0x00, 0x00, 0x00])
_SLG_STATUS_MODE1: Final[bytes] = bytes([0x01, 0x00, 0x00, 0x00])
_MDS_STATUS_MODE3: Final[bytes] = bytes([0x03, 0x00, 0x00])

_RESET_BODY_LEN: Final[int] = 3
_SLG_MODE0_BODY_LEN: Final[int] = 3
_SLG_MODE1_BODY_LEN: Final[int] = 25
_MDS_MODE3_BODY_LEN: Final[int] = 16
_ANW_MELD_BODY_LEN: Final[int] = 2

_SETTLE_POLL_SEC: Final[float] = 0.05
_THREAD_JOIN_SEC: Final[float] = 2.0

_BAUD_CODES: Final[dict[int, int]] = {0x01: 19200, 0x03: 57600, 0x05: 115200}
_PARITY_CODES: Final[dict[int, str]] = {0b00: 'odd', 0b01: 'even', 0b10: 'none'}
_INTERFACE_CODES: Final[dict[int, str]] = {0x01: 'RS422', 0x02: 'RS232'}

_RESYNC = object()
_STOP = object()


class _PendingCommand:
    """The single in-flight command waiting for its acknowledgement."""

    def __init__(self, command: AsciiCommandEnum) -> None:
        self.command = command
        self.event = threading.Event()
        self.frame: AsciiFrameDTO | None = None
        self.error: str | None = None


class SimaticAsciiReader(RfidReader):
    """Real SIMATIC RF240R reader (6GT2821-4AC40) using the ASCII protocol."""

    def __init__(
        self,
        node: LifecycleNode,
        config: NodeConfigDTO,
        on_detected_changed: Callable[[bool], None],
        on_fatal_error: Callable[[str], None],
    ) -> None:
        """Store collaborators; no resources are acquired here."""
        super().__init__(node, config, on_detected_changed, on_fatal_error)
        self._serial: serial.Serial | None = None
        self._rx_thread: threading.Thread | None = None
        self._presence_thread: threading.Thread | None = None
        self._presence_queue: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._cmd_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: _PendingCommand | None = None
        self._stale_codes: set[int] = set()
        self._heartbeat_timer: Timer | None = None
        self._unusable_presence = False

    # region lifecycle
    def initialize(self) -> None:
        """Open the port, reset the reader and start presence monitoring."""
        cfg = self._config
        with self._state_lock:
            self._tag_info = None
            self._fatal_reported = False
            self._fatal_reporting_enabled = False
        self._unusable_presence = False
        self._stale_codes.clear()
        self._stop.clear()
        self._presence_queue = queue.Queue()

        self._serial = serial.Serial(
            port=cfg.serial_port,
            baudrate=cfg.baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=cfg.parity.to_pyserial(),
            stopbits=serial.STOPBITS_ONE if cfg.stop_bits == 1 else serial.STOPBITS_TWO,
            timeout=0.1,
            write_timeout=cfg.response_timeout_sec,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='rf240r_rx', daemon=True)
        self._rx_thread.start()
        self._presence_thread = threading.Thread(
            target=self._presence_loop, name='rf240r_presence', daemon=True)
        self._presence_thread.start()

        with self._cmd_lock:
            self._send_command(
                AsciiCommandEnum.RESET, _RESET_PARAMS,
                validator=self._body_len_validator(_RESET_BODY_LEN))
            identity = self._send_command(
                AsciiCommandEnum.SLG_STATUS, _SLG_STATUS_MODE1,
                validator=self._slg_validator(1, _SLG_MODE1_BODY_LEN))
            self._log_identity(identity.body)
            _, present = self._slg_status_mode0()
            if present:
                self._tag_arrived_locked(raise_fatal=True)

        self._heartbeat_timer = self._node.create_timer(
            cfg.heartbeat_period_sec, self._on_heartbeat,
            callback_group=MutuallyExclusiveCallbackGroup())
        with self._state_lock:
            self._fatal_reporting_enabled = True
        self._logger.info(
            f"SIMATIC RF240R ready on '{cfg.serial_port}' "
            f'({cfg.baud_rate} Bd, 8{cfg.parity.value[0].upper()}{cfg.stop_bits})')

    def deinitialize(self) -> None:
        """Stop all threads, close the port and clear the state. Never raises."""
        with self._state_lock:
            self._fatal_reporting_enabled = False
        if self._heartbeat_timer is not None:
            try:
                self._node.destroy_timer(self._heartbeat_timer)
            except Exception as e:
                self._logger.warning(f'Failed to destroy heartbeat timer: {e}')
            self._heartbeat_timer = None

        self._stop.set()
        self._presence_queue.put(_STOP)
        self._fail_pending('reader deinitialised')

        if self._serial is not None:
            try:
                self._serial.close()
            except Exception as e:
                self._logger.warning(f'Failed to close serial port: {e}')

        for thread in (self._rx_thread, self._presence_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=_THREAD_JOIN_SEC)
                if thread.is_alive():
                    self._logger.warning(f'Thread {thread.name} did not stop in time')
        self._rx_thread = None
        self._presence_thread = None
        self._serial = None

        with self._pending_lock:
            self._pending = None
            self._stale_codes.clear()
        with self._state_lock:
            self._tag_info = None
        self._unusable_presence = False
    # endregion lifecycle

    # region read / write
    def _read_impl(self, tag: TagInfoDTO, start_address: int, length: int) -> bytes:
        """Send a single READ telegram."""
        address = start_address.to_bytes(2, 'big')

        def validate(frame: AsciiFrameDTO) -> None:
            body = frame.body
            if len(body) != 3 + length:
                raise FrameError(f'READ body length {len(body)}, expected {3 + length}')
            if body[0:2] != address or body[2] != length:
                raise FrameError('READ acknowledgement does not echo address/length')

        with self._cmd_lock:
            frame = self._send_command(
                AsciiCommandEnum.READ, address + bytes([length]), validator=validate)
        return bytes(frame.body[3:])

    def _write_impl(self, tag: TagInfoDTO, start_address: int, data: bytes) -> None:
        """Send a single WRITE telegram."""
        params = start_address.to_bytes(2, 'big') + bytes([len(data)]) + bytes(data)
        with self._cmd_lock:
            self._send_command(
                AsciiCommandEnum.WRITE, params, validator=self._body_len_validator(0))
    # endregion read / write

    # region command handling
    def _send_command(
        self,
        command: AsciiCommandEnum,
        params: bytes,
        validator: Callable[[AsciiFrameDTO], None] | None = None,
        resync_on_presence_error: bool = True,
    ) -> AsciiFrameDTO:
        """Send one command and wait for its acknowledgement (caller holds _cmd_lock)."""
        is_status_cmd = command in (AsciiCommandEnum.SLG_STATUS, AsciiCommandEnum.RESET)
        if self._stale_codes and not is_status_cmd:
            self._settle()

        ser = self._serial
        if ser is None or self._stop.is_set():
            raise ReaderFatalError('reader not initialised')

        pending = _PendingCommand(command)
        with self._pending_lock:
            self._pending = pending
        try:
            ser.write(encode_frame(command, params))
        except Exception as e:
            with self._pending_lock:
                self._pending = None
            raise self._fatal_error(f'serial write failed: {e}') from e

        pending.event.wait(self._config.response_timeout_sec)
        with self._pending_lock:
            if self._pending is pending:
                self._pending = None
            answered = pending.frame is not None or pending.error is not None
            if not answered and not is_status_cmd:
                # The late acknowledgement must be discarded, not treated as unexpected.
                self._stale_codes.add(command.code)
        if pending.error is not None:
            raise ReaderFatalError(pending.error)
        if not answered:
            raise CommandTimeoutError(
                f'no acknowledgement for {command.name} within '
                f'{self._config.response_timeout_sec} s')

        frame = pending.frame
        assert frame is not None
        if frame.status == ReaderErrorEnum.OK.code and validator is not None:
            try:
                validator(frame)
            except FrameError as e:
                raise self._fatal_error(f'invalid {command.name} acknowledgement: {e}') from e
        self._check_status(command, frame.status, resync_on_presence_error)
        return frame

    def _check_status(
        self, command: AsciiCommandEnum, status: int, resync_on_presence_error: bool,
    ) -> None:
        """Map a non-zero error_Moby status to an exception."""
        if status == ReaderErrorEnum.OK.code:
            return
        error = ReaderErrorEnum.from_code(status)
        if error is None:
            raise TagAccessError(f'unknown reader error 0x{status:02X}')
        if error.is_fatal:
            raise self._fatal_error(
                f'{error.description} (0x{status:02X}) on {command.name}')
        if error is ReaderErrorEnum.ADDRESS_ERROR:
            raise TagAccessError(
                'out of bounds: reader reported address error (0x0D)', error)
        if error is ReaderErrorEnum.PRESENCE_ERROR and resync_on_presence_error:
            self._presence_queue.put(_RESYNC)
        raise TagAccessError(f'{error.description} (0x{status:02X})', error)

    def _settle(self) -> None:
        """Wait until the reader reports no active command, then clear stale codes."""
        deadline = time.monotonic() + self._config.response_timeout_sec
        while True:
            try:
                command_active, _ = self._slg_status_mode0()
            except CommandTimeoutError as e:
                raise self._fatal_error(f'SLG-STATUS timeout while settling: {e}') from e
            if not command_active:
                return
            if time.monotonic() >= deadline:
                raise TagAccessError('previous command still active')
            time.sleep(_SETTLE_POLL_SEC)

    def _slg_status_mode0(self) -> tuple[bool, bool]:
        """Return (command_active, tag_present) (caller holds _cmd_lock)."""
        frame = self._send_command(
            AsciiCommandEnum.SLG_STATUS, _SLG_STATUS_MODE0,
            validator=self._slg_validator(0, _SLG_MODE0_BODY_LEN))
        command_active = frame.body[1] != 0x00
        present = frame.body[2] == 0x01
        if not command_active:
            with self._pending_lock:
                self._stale_codes.clear()
        return command_active, present

    @staticmethod
    def _body_len_validator(expected: int) -> Callable[[AsciiFrameDTO], None]:
        """Return a validator checking the acknowledgement body length."""
        def validate(frame: AsciiFrameDTO) -> None:
            if len(frame.body) != expected:
                raise FrameError(f'body length {len(frame.body)}, expected {expected}')
        return validate

    @staticmethod
    def _slg_validator(mode: int, expected: int) -> Callable[[AsciiFrameDTO], None]:
        """Return a validator for an SLG-STATUS acknowledgement of a given mode."""
        def validate(frame: AsciiFrameDTO) -> None:
            if len(frame.body) != expected:
                raise FrameError(f'body length {len(frame.body)}, expected {expected}')
            if frame.body[0] != mode:
                raise FrameError(f'SLG-STATUS mode {frame.body[0]}, expected {mode}')
        return validate
    # endregion command handling

    # region receive path
    def _rx_loop(self) -> None:
        """Read the serial stream and route complete frames."""
        assembler = FrameAssembler()
        ser = self._serial
        while not self._stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except Exception as e:
                if not self._stop.is_set():
                    self._fatal(f'serial read failed: {e}')
                return
            if not chunk:
                continue
            try:
                for payload in assembler.feed(chunk):
                    self._route(decode_frame(payload))
            except FrameError as e:
                self._fatal(f'malformed frame: {e}')
                return

    def _route(self, frame: AsciiFrameDTO) -> None:
        """Deliver a frame to the presence handler or the waiting command."""
        if frame.command == AsciiCommandEnum.ANW_MELD.code:
            if frame.status != 0 or len(frame.body) != _ANW_MELD_BODY_LEN:
                self._fatal(f'malformed ANW-MELD frame: {frame}')
                return
            self._presence_queue.put(frame.body[1] == 0x01)
            return

        with self._pending_lock:
            pending = self._pending
            if (pending is not None and pending.frame is None and pending.error is None
                    and pending.command.code == frame.command):
                pending.frame = frame
                pending.event.set()
                return
            if frame.command in self._stale_codes:
                self._stale_codes.discard(frame.command)
                self._logger.debug(
                    f'Discarded late acknowledgement for command 0x{frame.command:02X}')
                return
        self._fatal(
            f'unexpected frame: command 0x{frame.command:02X}, status 0x{frame.status:02X}')
    # endregion receive path

    # region presence
    def _presence_loop(self) -> None:
        """Process presence jobs until stopped."""
        while True:
            job = self._presence_queue.get()
            if job is _STOP or self._stop.is_set():
                return
            try:
                if job is _RESYNC:
                    self._resync_presence()
                else:
                    self._handle_presence(bool(job))
            except ReaderFatalError:
                pass
            except Exception as e:
                self._fatal(f'presence handling failed: {e}')

    def _handle_presence(self, present: bool) -> None:
        """Handle a tag arrival or departure."""
        if not present:
            self._unusable_presence = False
            self._set_detected(None)
            return
        if self.is_tag_detected or self._unusable_presence:
            return
        with self._cmd_lock:
            if self.is_tag_detected or self._stop.is_set():
                return
            self._tag_arrived_locked(raise_fatal=False)

    def _resync_presence(self) -> None:
        """Query the presence bit and apply it."""
        with self._cmd_lock:
            if self._stop.is_set():
                return
            try:
                _, present = self._slg_status_mode0()
            except CommandTimeoutError as e:
                self._fatal(f'SLG-STATUS timeout during presence re-sync: {e}')
                return
            except TagAccessError as e:
                self._logger.warning(f'Presence re-sync failed: {e}')
                return
        if present != self.is_tag_detected:
            self._handle_presence(present)

    def _tag_arrived_locked(self, raise_fatal: bool) -> None:
        """Read the tag's MDS-STATUS and mark it detected (caller holds _cmd_lock)."""
        try:
            frame = self._send_command(
                AsciiCommandEnum.MDS_STATUS, _MDS_STATUS_MODE3,
                validator=self._mds_validator, resync_on_presence_error=False)
        except ReaderFatalError:
            if raise_fatal:
                raise
            return
        except CommandTimeoutError as e:
            self._logger.warning(f'Tag arrival: MDS-STATUS timed out: {e}')
            return
        except TagAccessError as e:
            self._logger.info(f'Tag arrival: MDS-STATUS failed, staying not detected: {e}')
            return

        body = frame.body
        uid = body[1:9].hex().upper()
        tag_type = body[9]
        memory_size = int.from_bytes(body[11:13], 'big')
        if memory_size == 0:
            if not self._unusable_presence:
                self._logger.warning(
                    f'Tag {uid} reports user memory size 0; treating it as unusable')
            self._unusable_presence = True
            return
        self._logger.info(
            f'Tag detected: UID {uid}, type 0x{tag_type:02X}, {memory_size} bytes user memory')
        self._set_detected(
            TagInfoDTO(tag_id=uid, tag_type=tag_type, memory_size=memory_size))

    @staticmethod
    def _mds_validator(frame: AsciiFrameDTO) -> None:
        """Validate an MDS-STATUS mode 3 acknowledgement."""
        if len(frame.body) != _MDS_MODE3_BODY_LEN:
            raise FrameError(
                f'body length {len(frame.body)}, expected {_MDS_MODE3_BODY_LEN}')
        if frame.body[0] != 0x03:
            raise FrameError(f'MDS-STATUS mode {frame.body[0]}, expected 3')
    # endregion presence

    # region heartbeat
    def _on_heartbeat(self) -> None:
        """Poll SLG-STATUS mode 0; skipped while another command is in flight."""
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if self._stop.is_set():
                return
            try:
                _, present = self._slg_status_mode0()
            except CommandTimeoutError:
                self._fatal('heartbeat timeout')
                return
            except ReaderFatalError:
                return
            except TagAccessError as e:
                self._logger.warning(f'Heartbeat returned an error: {e}')
                return
        finally:
            self._cmd_lock.release()

        if present:
            if not self.is_tag_detected and not self._unusable_presence:
                self._presence_queue.put(True)
        elif self.is_tag_detected or self._unusable_presence:
            self._presence_queue.put(False)
    # endregion heartbeat

    # region helpers
    def _log_identity(self, body: bytes) -> None:
        """Log the SLG-STATUS mode 1 identity and warn about mismatches."""
        hw_type = body[1]
        interface = _INTERFACE_CODES.get(body[12], f'0x{body[12]:02X}')
        baud = _BAUD_CODES.get(body[13])
        parity = _PARITY_CODES.get((body[14] >> 2) & 0b11)
        stop_bits = 2 if body[14] & 0x01 else 1
        self._logger.info(
            f'Reader identity: HW type 0x{hw_type:02X}, '
            f'HW version {body[2]}.{body[3]}, loader {body[4]}.{body[5]}, '
            f'FW variant 0x{body[6]:02X} version {body[7]}.{body[8]}, '
            f'driver variant 0x{body[9]:02X} version {body[10]}.{body[11]}, '
            f'interface {interface}, baud {baud}, parity {parity}, stop bits {stop_bits}, '
            f'Ftim 0x{body[20]:02X}, antenna {"on" if body[22] == 0x01 else "off"}, '
            f'presence mode 0x{body[24]:02X}')
        if hw_type != RF240R_HW_TYPE:
            self._logger.warning(
                f'Reader hardware type 0x{hw_type:02X} is not an RF240R '
                f'(0x{RF240R_HW_TYPE:02X})')
        cfg = self._config
        if baud != cfg.baud_rate or parity != cfg.parity.value or stop_bits != cfg.stop_bits:
            self._logger.warning(
                f'Reader reports {baud} Bd, parity {parity}, {stop_bits} stop bit(s) but '
                f'the node is configured for {cfg.baud_rate} Bd, parity '
                f'{cfg.parity.value}, {cfg.stop_bits} stop bit(s)')

    def _fail_pending(self, reason: str) -> None:
        """Release the waiting command with an error."""
        with self._pending_lock:
            pending = self._pending
            if pending is not None and pending.frame is None and pending.error is None:
                pending.error = reason
                pending.event.set()

    def _fatal(self, reason: str) -> None:
        """Report a fatal error and release the waiting command immediately."""
        self._fail_pending(reason)
        self._report_fatal(reason)

    def _fatal_error(self, reason: str) -> ReaderFatalError:
        """Report a fatal error and return the exception to raise."""
        self._fatal(reason)
        return ReaderFatalError(reason)
    # endregion helpers
