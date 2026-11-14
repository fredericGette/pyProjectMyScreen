#!/usr/bin/env python3
"""pyProjectMyScreen - display the screen of a Windows Phone 8.1 device (Lumia 520)
connected by USB, and send touch events and hardware-button presses back to it.

Python re-implementation of the PC side of Microsoft "Project My Screen"
(ProjectMyScreenApp.exe). Protocol (see Lumia520/content/drivers/ufnvidstream.md):

* The phone exposes a USB interface bound to WinUSB on the PC, device interface
  GUID {31648E59-6FCB-4D81-9C97-D00999B0459A}, with a bulk IN pipe (frames) and a
  bulk OUT pipe (input messages).
* Class requests on the control pipe (bmRequestType 0x21, everything else 0):
  0x02 NOTIFY (start, phone asks for confirmation), 0x04 FORCE (restart),
  0x03 READY (before each frame read), 0x01 STOP (viewer closes).
* Frame = 512-byte header (magic 0x44555608, type 2, size, format) + RGB565 image.
* Input message = 192 bytes (magic, type 3, size 192, event type, touch contacts
  or hardware button).

Windows only (uses setupapi.dll / winusb.dll through ctypes).
Requires: pygame, numpy.
"""
from __future__ import annotations

import argparse
import ctypes
import logging
import queue
import struct
import sys
import threading
import time
import uuid
from collections import namedtuple

import numpy as np

log = logging.getLogger("pmys")

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
VIDSTREAM_GUID = "{31648E59-6FCB-4D81-9C97-D00999B0459A}"
MAGIC = 0x44555608
MSG_FRAME = 2
MSG_INPUT = 3
HEADER_SIZE = 0x200
INPUT_MSG_SIZE = 192

REQ_STOP = 0x01
REQ_NOTIFY = 0x02
REQ_READY = 0x03
REQ_FORCE = 0x04

EVENT_TOUCH = 0
EVENT_BUTTON = 1

BTN_BACK = 0
BTN_START = 1
BTN_SEARCH = 2
BUTTON_NAMES = {BTN_BACK: "Back", BTN_START: "Start", BTN_SEARCH: "Search"}

TOUCH_UP = 0
TOUCH_HOVER = 1
TOUCH_CONTACT = 2
MAX_CONTACTS = 10

# Largest frame the official app expects: 854 x 480 x 2 bytes + header.
MAX_FRAME_BYTES = 854 * 480 * 2 + HEADER_SIZE

# Resolutions guessed by the official app when the header has no line pitch.
GUESS_RESOLUTIONS = [(480, 800), (480, 854), (480, 640), (450, 800)]

# Header orientation (Windows DisplayOrientations) -> clockwise rotation (degrees)
# to apply to the received (portrait) image to show it upright on the PC.
# 2 = Landscape, 8 = LandscapeFlipped: direction is a best guess, use the
# --rotation option / 'R' key if the image appears upside down in landscape.
ORIENTATION_TO_CW = {0: 0, 1: 0, 2: 270, 4: 180, 8: 90}


# ---------------------------------------------------------------------------
# Frame parsing / input encoding (pure functions, testable without a phone)
# ---------------------------------------------------------------------------
class FrameError(Exception):
    pass


Frame = namedtuple("Frame", "rgb orientation screen_w screen_h img_w img_h softnav header")


def rgb565_to_rgb(px: np.ndarray) -> np.ndarray:
    """HxW uint16 RGB565 -> HxWx3 uint8 RGB (new array)."""
    r = ((px >> 11) & 0x1F).astype(np.uint8)
    g = ((px >> 5) & 0x3F).astype(np.uint8)
    b = (px & 0x1F).astype(np.uint8)
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)
    return np.dstack((r, g, b))


def _header_ok(data, off: int, n: int) -> bool:
    magic, msg_type, size = struct.unpack_from("<III", data, off)
    return magic == MAGIC and msg_type == MSG_FRAME and size == n - off


def find_frame_start(data, n: int) -> int:
    """Offset of the frame header in a read of `n` bytes.

    Normally 0. The phone sends the 512-byte header and the image as two
    separate transfers; when it drops a frame after its header, the orphan
    header(s) end up in front of the next frame in the same read. Such a read
    is recovered by looking for a valid header on each 512-byte boundary."""
    if n < HEADER_SIZE:
        raise FrameError(f"short read ({n} bytes)")
    for off in range(0, n - HEADER_SIZE + 1, HEADER_SIZE):
        if _header_ok(data, off, n):
            return off
    magic, msg_type, size = struct.unpack_from("<III", data, 0)
    if magic != MAGIC:
        raise FrameError(f"bad magic 0x{magic:08X} ({n} bytes)")
    if msg_type != MSG_FRAME:
        raise FrameError(f"bad message type {msg_type}")
    raise FrameError(f"size field {size} != bytes read {n}")


def parse_frame(data, n: int, prev_header: bytes | None = None) -> Frame:
    """Parse a frame read from the bulk IN pipe (`n` valid bytes in `data`).

    `prev_header` is the header of the previous frame: the phone sometimes sends
    a frame's header *after* its image, so a read can be a bare image of exactly
    the previous payload size; it is then decoded with the previous header."""
    try:
        base = find_frame_start(data, n)
    except FrameError:
        if prev_header is None or n != struct.unpack_from("<I", prev_header, 8)[0] - HEADER_SIZE:
            raise
        log.debug("image without header (%d bytes), reusing the previous header", n)
        return _decode_frame(prev_header, data, 0, n)
    if base:
        log.debug("skipped %d bytes of orphan header(s) before the frame", base)
    header = bytes(data[base:base + HEADER_SIZE])
    return _decode_frame(header, data, base + HEADER_SIZE, n - base - HEADER_SIZE)


def _decode_frame(header: bytes, data, pix_off: int, payload: int) -> Frame:
    bpp, stride, screen_w, screen_h, img_w, img_h, orientation = struct.unpack_from(
        "<IIHHhhI", header, 0x18)
    softnav = header[0x2C]
    if bpp != 16:
        raise FrameError(f"unsupported bpp {bpp}")
    if stride == 0 or img_w <= 0 or img_h <= 0:
        for w, h in GUESS_RESOLUTIONS:
            if w * h * 2 == payload:
                img_w, img_h, stride = w, h, w * 2
                break
        else:
            raise FrameError(f"cannot guess resolution from payload size {payload}")
    if stride % 2 or stride < img_w * 2 or stride * img_h > payload:
        raise FrameError(f"inconsistent format: stride {stride}, {img_w}x{img_h}, payload {payload}")
    px = np.frombuffer(data, dtype="<u2", count=(stride // 2) * img_h, offset=pix_off)
    px = px.reshape(img_h, stride // 2)[:, :img_w]
    if not screen_w or not screen_h:
        screen_w, screen_h = img_w, img_h
    return Frame(rgb565_to_rgb(px), orientation, screen_w, screen_h, img_w, img_h, softnav, header)


def _input_header(event_type: int) -> bytearray:
    msg = bytearray(INPUT_MSG_SIZE)
    struct.pack_into("<III", msg, 0, MAGIC, MSG_INPUT, INPUT_MSG_SIZE)
    struct.pack_into("<I", msg, 0x18, event_type)
    return msg


def encode_touch(contacts) -> bytes:
    """contacts: iterable of (id, state, prev_state, x, y), x/y in portrait phone pixels.
    Must contain ALL active contacts: the phone releases those not listed."""
    contacts = list(contacts)[:MAX_CONTACTS]
    msg = _input_header(EVENT_TOUCH)
    struct.pack_into("<I", msg, 0x1C, len(contacts))
    for i, (cid, state, prev, x, y) in enumerate(contacts):
        struct.pack_into("<HHIIHH", msg, 0x20 + 16 * i, cid, 0, state, prev, x, y)
    return bytes(msg)


def encode_button(button: int, pressed: bool) -> bytes:
    msg = _input_header(EVENT_BUTTON)
    struct.pack_into("<IB", msg, 0x1C, button, 1 if pressed else 0)
    return bytes(msg)


def rotate_image(rgb: np.ndarray, cw: int) -> np.ndarray:
    """Rotate an HxWx3 image clockwise by `cw` degrees (multiple of 90)."""
    return np.rot90(rgb, (-(cw // 90)) % 4)


def display_to_portrait(dx: float, dy: float, cw: int, w: int, h: int):
    """Map a pixel of the rotated (displayed) image back to the portrait image
    of size w x h. Inverse of rotate_image()."""
    k = (-(cw // 90)) % 4  # counter-clockwise quarter turns applied by np.rot90
    if k == 0:
        return dx, dy
    if k == 1:
        return w - 1 - dy, dx
    if k == 2:
        return w - 1 - dx, h - 1 - dy
    return dy, h - 1 - dx


# ---------------------------------------------------------------------------
# WinUSB access through ctypes
# ---------------------------------------------------------------------------
ERROR_FILE_NOT_FOUND = 2
ERROR_ACCESS_DENIED = 5
ERROR_GEN_FAILURE = 31
ERROR_SEM_TIMEOUT = 121
ERROR_INSUFFICIENT_BUFFER = 122
ERROR_NO_MORE_ITEMS = 259
ERROR_OPERATION_ABORTED = 995


class UsbError(Exception):
    def __init__(self, what: str, winerror: int):
        super().__init__(f"{what} failed: {ctypes.FormatError(winerror).strip()} ({winerror})")
        self.winerror = winerror


class DeviceNotFound(Exception):
    pass


if sys.platform == "win32":
    from ctypes import wintypes

    class GUID(ctypes.Structure):
        _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD),
                    ("Data3", wintypes.WORD), ("Data4", ctypes.c_ubyte * 8)]

        @classmethod
        def from_string(cls, text: str) -> "GUID":
            g = cls()
            ctypes.memmove(ctypes.byref(g), uuid.UUID(text).bytes_le, 16)
            return g

    class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
        _fields_ = [("cbSize", wintypes.DWORD), ("InterfaceClassGuid", GUID),
                    ("Flags", wintypes.DWORD), ("Reserved", ctypes.c_size_t)]

    class USB_INTERFACE_DESCRIPTOR(ctypes.Structure):
        _pack_ = 1
        _fields_ = [("bLength", ctypes.c_ubyte), ("bDescriptorType", ctypes.c_ubyte),
                    ("bInterfaceNumber", ctypes.c_ubyte), ("bAlternateSetting", ctypes.c_ubyte),
                    ("bNumEndpoints", ctypes.c_ubyte), ("bInterfaceClass", ctypes.c_ubyte),
                    ("bInterfaceSubClass", ctypes.c_ubyte), ("bInterfaceProtocol", ctypes.c_ubyte),
                    ("iInterface", ctypes.c_ubyte)]

    class WINUSB_PIPE_INFORMATION(ctypes.Structure):
        _fields_ = [("PipeType", ctypes.c_int), ("PipeId", ctypes.c_ubyte),
                    ("MaximumPacketSize", ctypes.c_ushort), ("Interval", ctypes.c_ubyte)]

    class WINUSB_SETUP_PACKET(ctypes.Structure):
        _pack_ = 1
        _fields_ = [("RequestType", ctypes.c_ubyte), ("Request", ctypes.c_ubyte),
                    ("Value", ctypes.c_ushort), ("Index", ctypes.c_ushort),
                    ("Length", ctypes.c_ushort)]

    _setupapi = ctypes.WinDLL("setupapi", use_last_error=True)
    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _winusb = ctypes.WinDLL("winusb", use_last_error=True)

    def _proto(dll, name, restype, *argtypes):
        f = getattr(dll, name)
        f.restype = restype
        f.argtypes = argtypes
        return f

    _HANDLE = ctypes.c_void_p
    _BOOL = wintypes.BOOL
    _PULONG = ctypes.POINTER(ctypes.c_ulong)
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    SetupDiGetClassDevsW = _proto(_setupapi, "SetupDiGetClassDevsW", _HANDLE,
                                  ctypes.POINTER(GUID), wintypes.LPCWSTR, wintypes.HWND, wintypes.DWORD)
    SetupDiEnumDeviceInterfaces = _proto(_setupapi, "SetupDiEnumDeviceInterfaces", _BOOL,
                                         _HANDLE, ctypes.c_void_p, ctypes.POINTER(GUID), wintypes.DWORD,
                                         ctypes.POINTER(SP_DEVICE_INTERFACE_DATA))
    SetupDiGetDeviceInterfaceDetailW = _proto(_setupapi, "SetupDiGetDeviceInterfaceDetailW", _BOOL,
                                              _HANDLE, ctypes.POINTER(SP_DEVICE_INTERFACE_DATA),
                                              ctypes.c_void_p, wintypes.DWORD,
                                              ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p)
    SetupDiDestroyDeviceInfoList = _proto(_setupapi, "SetupDiDestroyDeviceInfoList", _BOOL, _HANDLE)

    CreateFileW = _proto(_kernel32, "CreateFileW", _HANDLE, wintypes.LPCWSTR, wintypes.DWORD,
                         wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, _HANDLE)
    CloseHandle = _proto(_kernel32, "CloseHandle", _BOOL, _HANDLE)

    WinUsb_Initialize = _proto(_winusb, "WinUsb_Initialize", _BOOL, _HANDLE, ctypes.POINTER(_HANDLE))
    WinUsb_Free = _proto(_winusb, "WinUsb_Free", _BOOL, _HANDLE)
    WinUsb_QueryInterfaceSettings = _proto(_winusb, "WinUsb_QueryInterfaceSettings", _BOOL, _HANDLE,
                                           ctypes.c_ubyte, ctypes.POINTER(USB_INTERFACE_DESCRIPTOR))
    WinUsb_QueryPipe = _proto(_winusb, "WinUsb_QueryPipe", _BOOL, _HANDLE, ctypes.c_ubyte,
                              ctypes.c_ubyte, ctypes.POINTER(WINUSB_PIPE_INFORMATION))
    WinUsb_SetPipePolicy = _proto(_winusb, "WinUsb_SetPipePolicy", _BOOL, _HANDLE, ctypes.c_ubyte,
                                  ctypes.c_ulong, ctypes.c_ulong, ctypes.c_void_p)
    WinUsb_GetPipePolicy = _proto(_winusb, "WinUsb_GetPipePolicy", _BOOL, _HANDLE, ctypes.c_ubyte,
                                  ctypes.c_ulong, _PULONG, ctypes.c_void_p)
    WinUsb_ControlTransfer = _proto(_winusb, "WinUsb_ControlTransfer", _BOOL, _HANDLE,
                                    WINUSB_SETUP_PACKET, ctypes.c_void_p, ctypes.c_ulong, _PULONG,
                                    ctypes.c_void_p)
    WinUsb_ReadPipe = _proto(_winusb, "WinUsb_ReadPipe", _BOOL, _HANDLE, ctypes.c_ubyte,
                             ctypes.c_void_p, ctypes.c_ulong, _PULONG, ctypes.c_void_p)
    WinUsb_WritePipe = _proto(_winusb, "WinUsb_WritePipe", _BOOL, _HANDLE, ctypes.c_ubyte,
                              ctypes.c_void_p, ctypes.c_ulong, _PULONG, ctypes.c_void_p)
    WinUsb_AbortPipe = _proto(_winusb, "WinUsb_AbortPipe", _BOOL, _HANDLE, ctypes.c_ubyte)
    WinUsb_ResetPipe = _proto(_winusb, "WinUsb_ResetPipe", _BOOL, _HANDLE, ctypes.c_ubyte)

DIGCF_PRESENT = 0x02
DIGCF_DEVICEINTERFACE = 0x10
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x01
FILE_SHARE_WRITE = 0x02
OPEN_EXISTING = 3
FILE_ATTRIBUTE_NORMAL = 0x80
FILE_FLAG_OVERLAPPED = 0x40000000
USBD_PIPE_TYPE_BULK = 2
PIPE_TRANSFER_TIMEOUT = 0x03
RAW_IO = 0x07
MAXIMUM_TRANSFER_SIZE = 0x08


def _check(ok, what: str):
    if not ok:
        raise UsbError(what, ctypes.get_last_error())


class WinUsbDevice:
    """Minimal synchronous WinUSB wrapper for the VidStream interface."""

    def __init__(self, path: str):
        self.path = path
        self.file = None
        self.handle = ctypes.c_void_p()
        h = CreateFileW(path, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                        None, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, None)
        if h is None or h == INVALID_HANDLE_VALUE:
            raise UsbError("CreateFile", ctypes.get_last_error())
        self.file = h
        try:
            _check(WinUsb_Initialize(self.file, ctypes.byref(self.handle)), "WinUsb_Initialize")
            self._query_pipes()
            self._set_policies()
        except Exception:
            self.close()
            raise

    @staticmethod
    def find_path(guid_text: str) -> str:
        guid = GUID.from_string(guid_text)
        devs = SetupDiGetClassDevsW(ctypes.byref(guid), None, None, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE)
        if devs is None or devs == INVALID_HANDLE_VALUE:
            raise UsbError("SetupDiGetClassDevs", ctypes.get_last_error())
        try:
            ifdata = SP_DEVICE_INTERFACE_DATA()
            ifdata.cbSize = ctypes.sizeof(SP_DEVICE_INTERFACE_DATA)
            if not SetupDiEnumDeviceInterfaces(devs, None, ctypes.byref(guid), 0, ctypes.byref(ifdata)):
                err = ctypes.get_last_error()
                if err == ERROR_NO_MORE_ITEMS:
                    raise DeviceNotFound("no device with the VidStream interface")
                raise UsbError("SetupDiEnumDeviceInterfaces", err)
            required = wintypes.DWORD()
            SetupDiGetDeviceInterfaceDetailW(devs, ctypes.byref(ifdata), None, 0, ctypes.byref(required), None)
            if required.value == 0:
                raise UsbError("SetupDiGetDeviceInterfaceDetail", ctypes.get_last_error())
            buf = ctypes.create_string_buffer(required.value)
            # SP_DEVICE_INTERFACE_DETAIL_DATA_W.cbSize: 8 on 64-bit, 6 on 32-bit
            struct.pack_into("<I", buf, 0, 8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6)
            _check(SetupDiGetDeviceInterfaceDetailW(devs, ctypes.byref(ifdata), buf, required,
                                                    None, None), "SetupDiGetDeviceInterfaceDetail")
            return ctypes.wstring_at(ctypes.addressof(buf) + 4)
        finally:
            SetupDiDestroyDeviceInfoList(devs)

    def _query_pipes(self):
        desc = USB_INTERFACE_DESCRIPTOR()
        _check(WinUsb_QueryInterfaceSettings(self.handle, 0, ctypes.byref(desc)),
               "WinUsb_QueryInterfaceSettings")
        self.pipe_in = self.pipe_out = None
        for i in range(desc.bNumEndpoints):
            info = WINUSB_PIPE_INFORMATION()
            _check(WinUsb_QueryPipe(self.handle, 0, i, ctypes.byref(info)), "WinUsb_QueryPipe")
            log.debug("pipe %d: type %d id 0x%02X max packet %d", i, info.PipeType, info.PipeId,
                      info.MaximumPacketSize)
            if info.PipeType != USBD_PIPE_TYPE_BULK:
                continue
            if info.PipeId & 0x80 and self.pipe_in is None:
                self.pipe_in, self.packet_size = info.PipeId, info.MaximumPacketSize
            elif not info.PipeId & 0x80 and self.pipe_out is None:
                self.pipe_out = info.PipeId
        if self.pipe_in is None or self.pipe_out is None:
            raise UsbError("USB device doesn't advertise the proper interface", ERROR_GEN_FAILURE)
        # Read size rounded up to the packet size (needed by RAW_IO).
        self.read_size = -(-MAX_FRAME_BYTES // self.packet_size) * self.packet_size

    def _set_policy(self, pipe: int, policy: int, value: int, ctype=ctypes.c_ulong):
        v = ctype(value)
        _check(WinUsb_SetPipePolicy(self.handle, pipe, policy, ctypes.sizeof(v), ctypes.byref(v)),
               f"WinUsb_SetPipePolicy(0x{pipe:02X}, {policy})")

    def _set_policies(self):
        self._set_policy(0x00, PIPE_TRANSFER_TIMEOUT, 2000)
        self._set_policy(self.pipe_in, PIPE_TRANSFER_TIMEOUT, 2500)
        self._set_policy(self.pipe_out, PIPE_TRANSFER_TIMEOUT, 1000)
        max_size = ctypes.c_ulong()
        length = ctypes.c_ulong(ctypes.sizeof(max_size))
        if WinUsb_GetPipePolicy(self.handle, self.pipe_in, MAXIMUM_TRANSFER_SIZE, ctypes.byref(length),
                                ctypes.byref(max_size)) and self.read_size <= max_size.value:
            self._set_policy(self.pipe_in, RAW_IO, 1, ctypes.c_ubyte)
        else:
            log.debug("RAW_IO not enabled (max transfer size %d)", max_size.value)

    def control(self, request: int):
        setup = WINUSB_SETUP_PACKET(0x21, request, 0, 0, 0)
        done = ctypes.c_ulong()
        _check(WinUsb_ControlTransfer(self.handle, setup, None, 0, ctypes.byref(done), None),
               f"class request 0x{request:02X}")

    def read(self, buf) -> int:
        done = ctypes.c_ulong()
        _check(WinUsb_ReadPipe(self.handle, self.pipe_in, buf, len(buf), ctypes.byref(done), None),
               "WinUsb_ReadPipe")
        return done.value

    def write(self, data: bytes) -> int:
        buf = ctypes.create_string_buffer(data, len(data))
        done = ctypes.c_ulong()
        _check(WinUsb_WritePipe(self.handle, self.pipe_out, buf, len(data), ctypes.byref(done), None),
               "WinUsb_WritePipe")
        return done.value

    def abort(self):
        for pipe in (self.pipe_in, self.pipe_out):
            WinUsb_AbortPipe(self.handle, pipe)

    def reset_pipes(self):
        for pipe in (self.pipe_in, self.pipe_out):
            WinUsb_AbortPipe(self.handle, pipe)
            WinUsb_ResetPipe(self.handle, pipe)

    def close(self):
        if self.handle:
            WinUsb_Free(self.handle)
            self.handle = ctypes.c_void_p()
        if self.file:
            CloseHandle(self.file)
            self.file = None


# ---------------------------------------------------------------------------
# Protocol client (reader + writer threads)
# ---------------------------------------------------------------------------
class VidStreamClient:
    def __init__(self, guid: str = VIDSTREAM_GUID):
        self.guid = guid
        self.status = "Starting"
        self.streaming = False
        self.frame_seq = 0
        self._frame = None
        self._frame_lock = threading.Lock()
        self._dev = None
        self._dev_lock = threading.Lock()
        self._out = queue.Queue(maxsize=32)
        self._stop = threading.Event()
        self._notified = False
        self._threads = []

    # -- public API (called from the UI thread) --
    def start(self):
        for target, name in ((self._reader, "vidstream-reader"), (self._writer, "vidstream-writer")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self):
        self._stop.set()
        with self._dev_lock:
            if self._dev is not None:
                self._dev.abort()
        for t in self._threads:
            t.join(timeout=5)

    def latest_frame(self):
        with self._frame_lock:
            return self.frame_seq, self._frame

    def send(self, msg: bytes) -> bool:
        if not self.streaming:
            return False
        try:
            self._out.put_nowait(msg)
            return True
        except queue.Full:
            log.warning("input queue full, message dropped")
            return False

    # -- threads --
    def _reader(self):
        last_status = None
        while not self._stop.is_set():
            try:
                path = WinUsbDevice.find_path(self.guid)
                dev = WinUsbDevice(path)
            except DeviceNotFound:
                self.status = "Waiting for the phone (connect it by USB, screen unlocked)"
            except UsbError as e:
                if e.winerror == ERROR_ACCESS_DENIED:
                    self.status = "Device busy (is ProjectMyScreenApp running?)"
                else:
                    self.status = f"Cannot open the phone: {e}"
            else:
                log.info("opened %s (packet size %d)", path, dev.packet_size)
                with self._dev_lock:
                    self._dev = dev
                try:
                    self._session(dev)
                except UsbError as e:
                    if not self._stop.is_set():
                        log.warning("%s", e)
                        self.status = f"Connection lost: {e}"
                finally:
                    self.streaming = False
                    if self._stop.is_set():
                        try:
                            dev.control(REQ_STOP)
                            log.info("sent STOP")
                        except UsbError as e:
                            log.debug("STOP: %s", e)
                    with self._dev_lock:
                        self._dev = None
                        dev.close()
            if self._stop.is_set():
                break
            if self.status != last_status:
                log.info("%s", self.status)
                last_status = self.status
            self._stop.wait(1.0)

    def _session(self, dev: WinUsbDevice):
        if self._notified:
            try:
                dev.control(REQ_FORCE)
                log.info("sent FORCE")
            except UsbError:
                dev.control(REQ_NOTIFY)
                log.info("sent NOTIFY")
        else:
            dev.control(REQ_NOTIFY)
            log.info("sent NOTIFY")
            self._notified = True
        dev.reset_pipes()
        self.status = "Accept the projection prompt on the phone"
        log.info("%s", self.status)
        buf = ctypes.create_string_buffer(dev.read_size)
        bad = 0
        while not self._stop.is_set():
            dev.control(REQ_READY)
            try:
                n = dev.read(buf)
            except UsbError as e:
                if e.winerror == ERROR_SEM_TIMEOUT:
                    continue  # no frame for 2.5 s: send READY again
                raise
            try:
                frame = parse_frame(buf, n, self._frame.header if self._frame else None)
            except FrameError as e:
                # Each read ends on a transfer boundary, so the next read is
                # normally aligned again: only reset after repeated failures.
                bad += 1
                log.info("bad frame dropped: %s", e)
                if bad >= 5:
                    log.warning("%d bad frames in a row, resetting the pipes", bad)
                    dev.reset_pipes()
                    bad = 0
                continue
            bad = 0
            with self._frame_lock:
                self._frame = frame
                self.frame_seq += 1
            if not self.streaming:
                log.info("streaming %dx%d (screen %dx%d, orientation %d)", frame.img_w, frame.img_h,
                         frame.screen_w, frame.screen_h, frame.orientation)
                self.streaming = True
                self.status = "Streaming"

    def _writer(self):
        while not self._stop.is_set():
            try:
                msg = self._out.get(timeout=0.2)
            except queue.Empty:
                continue
            with self._dev_lock:
                dev = self._dev
                if dev is None or not self.streaming:
                    continue
                try:
                    n = dev.write(msg)
                    if n != INPUT_MSG_SIZE:
                        log.warning("input message: %d bytes written", n)
                except UsbError as e:
                    log.warning("input message: %s", e)


# ---------------------------------------------------------------------------
# pygame viewer
# ---------------------------------------------------------------------------
class Viewer:
    BAR_H = 56
    BG = (24, 24, 28)
    TOUCH_INTERVAL = 0.012   # min delay between two position updates (like the official app)
    KEEPALIVE = 0.5          # resend held contacts (the phone releases them after 1 s idle)

    def __init__(self, client: VidStreamClient, scale: float | None, rotation: int | None):
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.display.set_caption("pyProjectMyScreen")
        self.client = client
        self.rotation_override = rotation
        self.font = pygame.font.SysFont("segoeui,arial", 18)
        self.small = pygame.font.SysFont("segoeui,arial", 15)

        self.img_size = (480, 800)       # portrait image size (from the frames)
        self.screen_res = (480, 800)     # phone screen resolution (from the frames)
        self.cw = 0                      # clockwise rotation applied to the image
        self.surface = None
        self.scaled = None
        self.shown_seq = -1

        desk_h = pygame.display.get_desktop_sizes()[0][1]
        self.scale = scale if scale else min(1.0, (desk_h * 0.9 - self.BAR_H - 40) / 800)
        self.fullscreen = False
        self._set_window()

        self.contacts = {}      # contact id -> [state, prev_state, x, y]
        self.finger_cid = {}    # SDL finger id -> contact id
        self.finger_bar = {}    # SDL finger id -> button pressed on the bar
        self.mouse_cid = None
        self.mouse_bar = None
        self.dirty = False
        self.last_touch_send = 0.0
        self.held_button = None

        self.fps = 0.0
        self._fps_count = 0
        self._fps_time = time.monotonic()
        self._fps_seq = 0

    # -- window / layout --
    def _display_size(self):
        w, h = self.img_size
        return (h, w) if self.cw in (90, 270) else (w, h)

    def _set_window(self):
        pg = self.pg
        if self.fullscreen:
            self.screen = pg.display.set_mode((0, 0), pg.FULLSCREEN)
        else:
            dw, dh = self._display_size()
            self.screen = pg.display.set_mode((int(dw * self.scale), int(dh * self.scale) + self.BAR_H),
                                              pg.RESIZABLE)
        self.scaled = None

    def _layout(self):
        pg = self.pg
        W, H = self.screen.get_size()
        dw, dh = self._display_size()
        avail_h = max(1, H - self.BAR_H)
        s = min(W / dw, avail_h / dh)
        iw, ih = max(1, int(dw * s)), max(1, int(dh * s))
        self.img_rect = pg.Rect((W - iw) // 2, (avail_h - ih) // 2, iw, ih)
        bw = W / 3
        self.bar_rects = {b: pg.Rect(int(i * bw), H - self.BAR_H, int(bw), self.BAR_H)
                          for i, b in enumerate((BTN_BACK, BTN_START, BTN_SEARCH))}

    def _bar_hit(self, pos):
        for b, r in self.bar_rects.items():
            if r.collidepoint(pos):
                return b
        return None

    def window_to_phone(self, pos, clamp=False):
        """Window pixel -> portrait phone screen pixel, or None if outside the image."""
        r = self.img_rect
        if not clamp and not r.collidepoint(pos):
            return None
        dw, dh = self._display_size()
        dx = min(max((pos[0] - r.x) * dw / r.w, 0), dw - 1)
        dy = min(max((pos[1] - r.y) * dh / r.h, 0), dh - 1)
        iw, ih = self.img_size
        px, py = display_to_portrait(dx, dy, self.cw, iw, ih)
        sw, sh = self.screen_res
        x = min(max(int(round(px * sw / iw)), 0), sw - 1)
        y = min(max(int(round(py * sh / ih)), 0), sh - 1)
        return x, y

    # -- touch --
    def _alloc_cid(self):
        for cid in range(MAX_CONTACTS):
            if cid not in self.contacts:
                return cid
        return None

    def touch_down(self, cid, xy):
        c = self.contacts.get(cid)
        if c:
            c[0], c[2], c[3] = TOUCH_CONTACT, xy[0], xy[1]
        else:
            self.contacts[cid] = [TOUCH_CONTACT, TOUCH_UP, xy[0], xy[1]]
        self.flush_touch()

    def touch_hover(self, cid, xy):
        c = self.contacts.get(cid)
        if c:
            c[0], c[2], c[3] = TOUCH_HOVER, xy[0], xy[1]
            self.dirty = True
        else:
            self.contacts[cid] = [TOUCH_HOVER, TOUCH_UP, xy[0], xy[1]]
            self.flush_touch()

    def touch_move(self, cid, xy):
        c = self.contacts.get(cid)
        if c and (c[2], c[3]) != tuple(xy):
            c[2], c[3] = xy
            self.dirty = True

    def touch_up(self, cid, xy=None):
        c = self.contacts.get(cid)
        if c:
            c[0] = TOUCH_UP
            if xy:
                c[2], c[3] = xy
            self.flush_touch()

    def flush_touch(self):
        if not self.contacts:
            return
        items = [(cid, c[0], c[1], c[2], c[3]) for cid, c in sorted(self.contacts.items())]
        self.client.send(encode_touch(items))
        for cid, c in list(self.contacts.items()):
            c[1] = c[0]
            if c[0] == TOUCH_UP:
                del self.contacts[cid]
        self.dirty = False
        self.last_touch_send = time.monotonic()

    # -- buttons --
    def press_button(self, b):
        if self.held_button is not None and self.held_button != b:
            self.release_button(self.held_button)
        if self.held_button != b:
            self.client.send(encode_button(b, True))
            self.held_button = b
            log.debug("button %s pressed", BUTTON_NAMES[b])

    def release_button(self, b):
        if self.held_button == b:
            self.client.send(encode_button(b, False))
            self.held_button = None
            log.debug("button %s released", BUTTON_NAMES[b])

    def release_all(self):
        for c in self.contacts.values():
            c[0] = TOUCH_UP
        self.flush_touch()
        self.mouse_cid = None
        self.finger_cid.clear()
        self.finger_bar.clear()
        self.mouse_bar = None
        if self.held_button is not None:
            self.release_button(self.held_button)

    # -- events --
    def _key_button(self, key):
        pg = self.pg
        return {pg.K_ESCAPE: BTN_BACK, pg.K_BACKSPACE: BTN_BACK,
                pg.K_F2: BTN_START, pg.K_HOME: BTN_START,
                pg.K_F3: BTN_SEARCH}.get(key)

    def handle(self, e) -> bool:
        pg = self.pg
        if e.type == pg.QUIT:
            return False
        if e.type == pg.KEYDOWN:
            if e.key == pg.K_q and e.mod & pg.KMOD_CTRL:
                return False
            if e.key == pg.K_F11:
                self.release_all()
                self.fullscreen = not self.fullscreen
                self._set_window()
            elif e.key == pg.K_r:
                order = [None, 0, 90, 180, 270]
                self.rotation_override = order[(order.index(self.rotation_override) + 1) % len(order)]
                log.info("rotation: %s", "auto" if self.rotation_override is None
                         else f"{self.rotation_override} deg")
                self.shown_seq = -1
            else:
                b = self._key_button(e.key)
                if b is not None:
                    self.press_button(b)
        elif e.type == pg.KEYUP:
            b = self._key_button(e.key)
            if b is not None:
                self.release_button(b)
        elif e.type == pg.MOUSEBUTTONDOWN and e.button == 1 and not getattr(e, "touch", False):
            b = self._bar_hit(e.pos)
            if b is not None:
                self.mouse_bar = b
                self.press_button(b)
            else:
                xy = self.window_to_phone(e.pos)
                if xy:
                    if self.mouse_cid is None:
                        self.mouse_cid = self._alloc_cid()
                    if self.mouse_cid is not None:
                        self.touch_down(self.mouse_cid, xy)
        elif e.type == pg.MOUSEMOTION and not getattr(e, "touch", False):
            c = self.contacts.get(self.mouse_cid) if self.mouse_cid is not None else None
            if c and c[0] == TOUCH_CONTACT:
                self.touch_move(self.mouse_cid, self.window_to_phone(e.pos, clamp=True))
            elif not e.buttons[0]:
                xy = self.window_to_phone(e.pos)
                if xy and pg.key.get_mods() & pg.KMOD_CTRL:
                    if self.mouse_cid is None:
                        self.mouse_cid = self._alloc_cid()
                    if self.mouse_cid is not None:
                        self.touch_hover(self.mouse_cid, xy)
                elif c:
                    self.touch_up(self.mouse_cid)
                    self.mouse_cid = None
        elif e.type == pg.MOUSEBUTTONUP and e.button == 1 and not getattr(e, "touch", False):
            if self.mouse_bar is not None:
                self.release_button(self.mouse_bar)
                self.mouse_bar = None
            if self.mouse_cid is not None:
                self.touch_up(self.mouse_cid, self.window_to_phone(e.pos, clamp=True))
                self.mouse_cid = None
        elif e.type in (pg.FINGERDOWN, pg.FINGERMOTION, pg.FINGERUP):
            W, H = self.screen.get_size()
            pos = (e.x * W, e.y * H)
            if e.type == pg.FINGERDOWN:
                b = self._bar_hit(pos)
                if b is not None:
                    self.finger_bar[e.finger_id] = b
                    self.press_button(b)
                else:
                    xy = self.window_to_phone(pos)
                    cid = self._alloc_cid() if xy else None
                    if cid is not None:
                        self.finger_cid[e.finger_id] = cid
                        self.touch_down(cid, xy)
            elif e.type == pg.FINGERMOTION:
                cid = self.finger_cid.get(e.finger_id)
                if cid is not None:
                    self.touch_move(cid, self.window_to_phone(pos, clamp=True))
            else:
                b = self.finger_bar.pop(e.finger_id, None)
                if b is not None:
                    self.release_button(b)
                cid = self.finger_cid.pop(e.finger_id, None)
                if cid is not None:
                    self.touch_up(cid, self.window_to_phone(pos, clamp=True))
        elif e.type == pg.WINDOWFOCUSLOST:
            self.release_all()
        return True

    def tick(self):
        now = time.monotonic()
        if self.dirty and now - self.last_touch_send >= self.TOUCH_INTERVAL:
            self.flush_touch()
        elif self.contacts and now - self.last_touch_send >= self.KEEPALIVE:
            self.flush_touch()
        if not self.client.streaming and (self.contacts or self.held_button is not None):
            self.contacts.clear()
            self.finger_cid.clear()
            self.mouse_cid = None
            self.held_button = None

    # -- drawing --
    def _update_frame(self):
        seq, frame = self.client.latest_frame()
        if frame is None or seq == self.shown_seq:
            return
        self.shown_seq = seq
        if self.rotation_override is not None:
            cw = self.rotation_override
        else:
            cw = ORIENTATION_TO_CW.get(frame.orientation, 0)
        size = (frame.img_w, frame.img_h)
        if cw != self.cw or size != self.img_size:
            self.release_all()
            if not self.fullscreen and self.img_rect.w:
                dw, _ = self._display_size()
                self.scale = self.img_rect.w / dw
            self.cw, self.img_size = cw, size
            if not self.fullscreen:
                self._set_window()
            self._layout()
        self.screen_res = (frame.screen_w, frame.screen_h)
        rot = np.ascontiguousarray(rotate_image(frame.rgb, cw))
        self.surface = self.pg.image.frombuffer(rot.tobytes(), (rot.shape[1], rot.shape[0]), "RGB")
        self.scaled = None

    def draw(self):
        pg = self.pg
        self.screen.fill(self.BG)
        r = self.img_rect
        if self.surface is not None:
            if self.scaled is None or self.scaled.get_size() != r.size:
                self.scaled = pg.transform.smoothscale(self.surface, r.size)
            self.screen.blit(self.scaled, r)
        else:
            pg.draw.rect(self.screen, (0, 0, 0), r)
        if not self.client.streaming:
            shade = pg.Surface(r.size, pg.SRCALPHA)
            shade.fill((0, 0, 0, 170))
            self.screen.blit(shade, r)
            self._draw_text_wrapped(self.client.status, r)
        for b, br in self.bar_rects.items():
            color = (70, 110, 200) if b == self.held_button else (40, 40, 46)
            pg.draw.rect(self.screen, color, br.inflate(-6, -10), border_radius=8)
            label = self.font.render(BUTTON_NAMES[b], True, (230, 230, 230))
            self.screen.blit(label, label.get_rect(center=br.center))
        pg.display.flip()

    def _draw_text_wrapped(self, text, rect):
        words, lines, line = text.split(), [], ""
        for w in words:
            test = f"{line} {w}".strip()
            if self.font.size(test)[0] > rect.w - 30 and line:
                lines.append(line)
                line = w
            else:
                line = test
        lines.append(line)
        y = rect.centery - len(lines) * self.font.get_linesize() // 2
        for ln in lines:
            surf = self.font.render(ln, True, (240, 240, 240))
            self.screen.blit(surf, surf.get_rect(midtop=(rect.centerx, y)))
            y += self.font.get_linesize()

    def _update_caption(self):
        now = time.monotonic()
        if now - self._fps_time >= 1.0:
            seq = self.client.frame_seq
            self.fps = (seq - self._fps_seq) / (now - self._fps_time)
            self._fps_seq, self._fps_time = seq, now
            rot = "auto" if self.rotation_override is None else f"{self.rotation_override}"
            state = f"{self.fps:.0f} fps" if self.client.streaming else "not connected"
            self.pg.display.set_caption(f"pyProjectMyScreen - {state} - rotation {rot}")

    def run(self):
        clock = self.pg.time.Clock()
        self.img_rect = self.pg.Rect(0, 0, 0, 0)
        self._layout()
        running = True
        while running:
            for e in self.pg.event.get():
                if not self.handle(e):
                    running = False
                    break
            self._layout()
            self._update_frame()
            self.tick()
            self.draw()
            self._update_caption()
            clock.tick(60)
        self.release_all()
        time.sleep(0.1)  # let the writer thread send the releases
        self.pg.quit()


# ---------------------------------------------------------------------------
# Self test (no phone needed)
# ---------------------------------------------------------------------------
def selftest():
    w, h = 480, 800
    rng = np.random.default_rng(1)
    px = rng.integers(0, 65536, size=(h, w), dtype=np.uint16)
    # Frame with explicit format
    hdr = bytearray(HEADER_SIZE)
    struct.pack_into("<III", hdr, 0, MAGIC, MSG_FRAME, HEADER_SIZE + px.nbytes)
    struct.pack_into("<IIHHhhIB", hdr, 0x18, 16, w * 2, w, h, w, h, 2, 0)
    data = bytes(hdr) + px.astype("<u2").tobytes()
    f = parse_frame(data, len(data))
    assert (f.img_w, f.img_h, f.orientation) == (w, h, 2)
    assert (f.screen_w, f.screen_h) == (w, h)
    p = int(px[10, 20])
    assert tuple(f.rgb[10, 20]) == (((p >> 11) << 3) | ((p >> 11) >> 2),
                                    (((p >> 5) & 63) << 2) | (((p >> 5) & 63) >> 4),
                                    ((p & 31) << 3) | ((p & 31) >> 2))
    # Frame without line pitch -> resolution guessed
    struct.pack_into("<I", hdr, 0x1C, 0)
    struct.pack_into("<hh", hdr, 0x24, 0, 0)
    data = bytes(hdr) + px.astype("<u2").tobytes()
    f2 = parse_frame(data, len(data))
    assert (f2.img_w, f2.img_h) == (w, h) and np.array_equal(f.rgb, f2.rgb)
    # Orphan header(s) in front of a frame (seen on a real Lumia 520)
    for orphans in (1, 2):
        glued = bytes(hdr) * orphans + data
        f3 = parse_frame(glued, len(glued))
        assert np.array_equal(f.rgb, f3.rgb)
    # Image sent before its header: decoded with the previous header
    bare = data[HEADER_SIZE:]
    f4 = parse_frame(bare, len(bare), f.header)
    assert np.array_equal(f.rgb, f4.rgb)
    try:
        parse_frame(bare[:-512], len(bare) - 512, f.header)
        raise AssertionError("partial image accepted")
    except FrameError:
        pass
    # Bad frames
    for bad in (data[:100], b"\0" * len(data), data[HEADER_SIZE:]):
        try:
            parse_frame(bad, len(bad))
            raise AssertionError("bad frame accepted")
        except FrameError:
            pass
    # Input messages
    m = encode_touch([(0, TOUCH_CONTACT, TOUCH_UP, 100, 200), (1, TOUCH_UP, TOUCH_CONTACT, 5, 6)])
    assert len(m) == INPUT_MSG_SIZE
    assert struct.unpack_from("<IIII", m, 0)[:3] == (MAGIC, MSG_INPUT, INPUT_MSG_SIZE)
    assert struct.unpack_from("<II", m, 0x18) == (EVENT_TOUCH, 2)
    assert struct.unpack_from("<HHIIHH", m, 0x20) == (0, 0, 2, 0, 100, 200)
    assert struct.unpack_from("<HHIIHH", m, 0x30) == (1, 0, 0, 2, 5, 6)
    m = encode_button(BTN_SEARCH, True)
    assert len(m) == INPUT_MSG_SIZE and struct.unpack_from("<IIB", m, 0x18) == (EVENT_BUTTON, 2, 1)
    # Rotation round trip
    img = np.arange(w * h).reshape(h, w)
    for cw in (0, 90, 180, 270):
        rot = rotate_image(img, cw)
        for _ in range(200):
            dy, dx = rng.integers(0, rot.shape[0]), rng.integers(0, rot.shape[1])
            x, y = display_to_portrait(dx, dy, cw, w, h)
            assert rot[dy, dx] == img[int(y), int(x)], (cw, dx, dy)
    print("selftest OK")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Display the screen of a Windows Phone 8.1 device "
                                             "connected by USB and send touch / buttons back.")
    ap.add_argument("--scale", type=float, help="window scale (default: fit the desktop height)")
    ap.add_argument("--rotation", choices=["auto", "0", "90", "180", "270"], default="auto",
                    help="clockwise rotation of the image (default: from the phone orientation)")
    ap.add_argument("--guid", default=VIDSTREAM_GUID, help="device interface GUID of the VidStream interface")
    ap.add_argument("-v", "--verbose", action="count", default=0, help="-v info, -vv debug")
    ap.add_argument("--selftest", action="store_true", help="test parsing/encoding without a phone")
    args = ap.parse_args()

    logging.basicConfig(level=[logging.WARNING, logging.INFO, logging.DEBUG][min(args.verbose, 2)],
                        format="%(asctime)s %(threadName)s %(levelname)s %(message)s")
    if args.selftest:
        selftest()
        return
    if sys.platform != "win32":
        sys.exit("This script uses WinUSB and only runs on Windows.")

    client = VidStreamClient(args.guid)
    client.start()
    try:
        Viewer(client, args.scale, None if args.rotation == "auto" else int(args.rotation)).run()
    finally:
        client.stop()


if __name__ == "__main__":
    main()
