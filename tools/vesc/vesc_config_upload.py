#!/usr/bin/env python3
"""
VESC Configuration Uploader

Uploads motor (MCCONF) and app (APPCONF) configuration XML files to a VESC
motor controller via serial port, using the native VESC binary protocol.

Based on VESC Tool source code (configparams.cpp, vbytearray.cpp, packet.cpp,
commands.cpp, datatypes.h) for firmware 6.02.

Usage:
    python3 vesc_config_upload.py --motor /path/to/motor_config.xml
    python3 vesc_config_upload.py --app /path/to/app_config.xml
    python3 vesc_config_upload.py --motor motor.xml --app app.xml
    python3 vesc_config_upload.py --motor motor.xml --verify
    python3 vesc_config_upload.py --motor motor.xml --dry-run

Author: Generated from VESC Tool source
"""

import argparse
import math
import struct
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from ctypes import c_int8, c_int16, c_int32, c_uint8, c_uint16, c_uint32

# ---------------------------------------------------------------------------
# Constants from datatypes.h
# ---------------------------------------------------------------------------

CFG_T_UNDEFINED = 0
CFG_T_DOUBLE    = 1
CFG_T_INT       = 2
CFG_T_QSTRING   = 3
CFG_T_ENUM      = 4
CFG_T_BOOL      = 5
CFG_T_BITFIELD  = 6

VESC_TX_UNDEFINED    = 0
VESC_TX_UINT8        = 1
VESC_TX_INT8         = 2
VESC_TX_UINT16       = 3
VESC_TX_INT16        = 4
VESC_TX_UINT32       = 5
VESC_TX_INT32        = 6
VESC_TX_DOUBLE16     = 7
VESC_TX_DOUBLE32     = 8
VESC_TX_DOUBLE32_AUTO = 9

COMM_SET_MCCONF    = 13
COMM_GET_MCCONF    = 14
COMM_SET_APPCONF   = 16
COMM_GET_APPCONF   = 17

# ---------------------------------------------------------------------------
# CRC16 (CRC-CCITT XMODEM) - from packet.cpp
# ---------------------------------------------------------------------------

CRC16_TAB = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
]


def crc16(data: bytes) -> int:
    """CRC-CCITT (XMODEM) as used by VESC packet framing."""
    cksum = 0
    for b in data:
        cksum = CRC16_TAB[((cksum >> 8) ^ b) & 0xFF] ^ ((cksum << 8) & 0xFFFF)
    return cksum & 0xFFFF


# ---------------------------------------------------------------------------
# CRC32C - from utility.cpp  (Castagnoli polynomial 0x82F63B78)
# ---------------------------------------------------------------------------

def crc32c(data: bytes) -> int:
    """CRC32C used for config signature calculation."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc = crc ^ byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0x82F63B78 & mask)) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Byte encoding helpers - from vbytearray.cpp
# ---------------------------------------------------------------------------

def _round_double(x):
    """Match the C++ roundDouble: ceil(x-0.5) for negative, floor(x+0.5) for positive."""
    if x < 0.0:
        return math.ceil(x - 0.5)
    return math.floor(x + 0.5)


def encode_int8(val):
    return struct.pack('>b', c_int8(int(val)).value)

def encode_uint8(val):
    return struct.pack('>B', c_uint8(int(val)).value)

def encode_int16(val):
    return struct.pack('>h', c_int16(int(val)).value)

def encode_uint16(val):
    return struct.pack('>H', c_uint16(int(val)).value)

def encode_int32(val):
    return struct.pack('>i', c_int32(int(val)).value)

def encode_uint32(val):
    return struct.pack('>I', c_uint32(int(val)).value)

def encode_double16(number, scale):
    return encode_int16(int(_round_double(number * scale)))

def encode_double32(number, scale):
    return encode_int32(int(_round_double(number * scale)))

def encode_double32_auto(number):
    """
    Encodes a double into 4 bytes with custom mantissa/exponent format.
    Direct port of vbAppendDouble32Auto from vbytearray.cpp.
    """
    # Set subnormal numbers to 0
    if abs(number) < 1.5e-38:
        number = 0.0

    # Use single-precision frexp
    f_number = float(number)
    if f_number == 0.0:
        return encode_uint32(0)

    fr, e = math.frexp(f_number)
    fr_abs = abs(fr)
    fr_s = 0

    if fr_abs >= 0.5:
        fr_s = int((fr_abs - 0.5) * 2.0 * 8388608.0) & 0xFFFFFFFF
        e += 126

    res = ((e & 0xFF) << 23) | (fr_s & 0x7FFFFF)
    if fr < 0:
        res |= (1 << 31)

    return encode_uint32(res)

def encode_string(s):
    return s.encode('utf-8') + b'\x00'


def decode_int8(data, offset):
    return struct.unpack_from('>b', data, offset)[0], offset + 1

def decode_uint8(data, offset):
    return struct.unpack_from('>B', data, offset)[0], offset + 1

def decode_int16(data, offset):
    return struct.unpack_from('>h', data, offset)[0], offset + 2

def decode_uint16(data, offset):
    return struct.unpack_from('>H', data, offset)[0], offset + 2

def decode_int32(data, offset):
    return struct.unpack_from('>i', data, offset)[0], offset + 4

def decode_uint32(data, offset):
    return struct.unpack_from('>I', data, offset)[0], offset + 4

def decode_double16(data, offset, scale):
    val, offset = decode_int16(data, offset)
    return float(val) / scale, offset

def decode_double32(data, offset, scale):
    val, offset = decode_int32(data, offset)
    return float(val) / scale, offset

def decode_double32_auto(data, offset):
    """Decode a DOUBLE32_AUTO value. Port of vbPopFrontDouble32Auto."""
    res, offset = decode_uint32(data, offset)
    e = (res >> 23) & 0xFF
    fr = res & 0x7FFFFF
    negative = bool(res & (1 << 31))

    f = 0.0
    if e != 0 or fr != 0:
        f = float(fr) / (8388608.0 * 2.0) + 0.5
        e -= 126

    if negative:
        f = -f

    return math.ldexp(f, e), offset

def decode_string(data, offset):
    end = data.index(0, offset)
    return data[offset:end].decode('utf-8'), end + 1


# ---------------------------------------------------------------------------
# Parameter definition structures
# ---------------------------------------------------------------------------

class ParamDef:
    """Stores a parameter definition parsed from the FW parameter XML."""
    __slots__ = ('name', 'type', 'transmittable', 'vTx', 'vTxDoubleScale',
                 'enumNames', 'valDouble', 'valInt', 'valString')

    def __init__(self, name):
        self.name = name
        self.type = CFG_T_UNDEFINED
        self.transmittable = 0
        self.vTx = VESC_TX_UNDEFINED
        self.vTxDoubleScale = 1.0
        self.enumNames = []
        self.valDouble = 0.0
        self.valInt = 0
        self.valString = ''


def parse_param_definitions_xml(xml_path):
    """
    Parse a VESC Tool parameter definitions XML (e.g. parameters_mcconf.xml).
    Returns (params_dict, serialize_order) where:
      - params_dict: {name: ParamDef}
      - serialize_order: [name, ...] in serialization order
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()  # <ConfigParams>

    params = {}
    serialize_order = []

    # Parse <Params> section
    params_elem = root.find('Params')
    if params_elem is None:
        raise ValueError(f"No <Params> section found in {xml_path}")

    for child in params_elem:
        p = ParamDef(child.tag)

        type_elem = child.find('type')
        if type_elem is not None:
            p.type = int(type_elem.text)

        trans_elem = child.find('transmittable')
        if trans_elem is not None:
            p.transmittable = int(trans_elem.text)

        vtx_elem = child.find('vTx')
        if vtx_elem is not None:
            p.vTx = int(vtx_elem.text)

        vtx_scale_elem = child.find('vTxDoubleScale')
        if vtx_scale_elem is not None:
            p.vTxDoubleScale = float(vtx_scale_elem.text)

        val_double_elem = child.find('valDouble')
        if val_double_elem is not None:
            p.valDouble = float(val_double_elem.text)

        val_int_elem = child.find('valInt')
        if val_int_elem is not None:
            p.valInt = int(val_int_elem.text)

        val_string_elem = child.find('valString')
        if val_string_elem is not None and val_string_elem.text:
            p.valString = val_string_elem.text

        # Collect all enumNames in order
        for en in child.findall('enumNames'):
            if en.text:
                p.enumNames.append(en.text)

        params[child.tag] = p

    # Parse <SerOrder> section
    ser_order_elem = root.find('SerOrder')
    if ser_order_elem is None:
        raise ValueError(f"No <SerOrder> section found in {xml_path}")

    for ser_elem in ser_order_elem.findall('ser'):
        name = ser_elem.text.strip() if ser_elem.text else ''
        if name:
            serialize_order.append(name)

    return params, serialize_order


def parse_user_config_xml(xml_path, config_tag):
    """
    Parse a user config XML file (MCConfiguration or APPConfiguration).
    Returns {param_name: text_value}.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    if root.tag != config_tag:
        raise ValueError(
            f"Expected root tag <{config_tag}>, got <{root.tag}> in {xml_path}")

    values = {}
    for child in root:
        if child.tag == 'ConfigVersion':
            continue
        values[child.tag] = child.text if child.text else ''
    return values


# ---------------------------------------------------------------------------
# Signature calculation - from configparams.cpp getSignature()
# ---------------------------------------------------------------------------

def calc_signature(params, serialize_order):
    """
    Calculate the CRC32C signature of a config structure.
    Matches ConfigParams::getSignature() exactly.

    The signature string is built by concatenating, for each param in
    serialize_order:
      - param name
      - str(int(type))
      - str(int(vTx))
      - all enumNames joined
    Then CRC32C of the UTF-8 encoding.
    """
    sig_str = ''
    for name in serialize_order:
        p = params.get(name)
        if p is None:
            continue
        sig_str += name
        sig_str += str(int(p.type))
        sig_str += str(int(p.vTx))
        for en in p.enumNames:
            sig_str += en

    return crc32c(sig_str.encode('utf-8'))


# ---------------------------------------------------------------------------
# Serialization - from configparams.cpp serialize() + getParamSerial()
# ---------------------------------------------------------------------------

def serialize_param(p):
    """Serialize a single parameter value to bytes."""
    if p.type == CFG_T_DOUBLE:
        if p.vTx == VESC_TX_DOUBLE16:
            return encode_double16(p.valDouble, p.vTxDoubleScale)
        elif p.vTx == VESC_TX_DOUBLE32:
            return encode_double32(p.valDouble, p.vTxDoubleScale)
        elif p.vTx == VESC_TX_DOUBLE32_AUTO:
            return encode_double32_auto(p.valDouble)
        else:
            raise ValueError(f"Param {p.name}: unsupported vTx={p.vTx} for DOUBLE")

    elif p.type == CFG_T_INT:
        if p.vTx == VESC_TX_UINT8:
            return encode_uint8(p.valInt)
        elif p.vTx == VESC_TX_INT8:
            return encode_int8(p.valInt)
        elif p.vTx == VESC_TX_UINT16:
            return encode_uint16(p.valInt)
        elif p.vTx == VESC_TX_INT16:
            return encode_int16(p.valInt)
        elif p.vTx == VESC_TX_UINT32:
            return encode_uint32(p.valInt)
        elif p.vTx == VESC_TX_INT32:
            return encode_int32(p.valInt)
        else:
            raise ValueError(f"Param {p.name}: unsupported vTx={p.vTx} for INT")

    elif p.type == CFG_T_QSTRING:
        return encode_string(p.valString)

    elif p.type in (CFG_T_ENUM, CFG_T_BOOL, CFG_T_BITFIELD):
        # All three use vbAppendInt8
        return encode_int8(p.valInt)

    else:
        raise ValueError(f"Param {p.name}: unsupported type={p.type}")


def deserialize_param(p, data, offset):
    """Deserialize a single parameter value from bytes. Returns (value, new_offset)."""
    if p.type == CFG_T_DOUBLE:
        if p.vTx == VESC_TX_DOUBLE16:
            return decode_double16(data, offset, p.vTxDoubleScale)
        elif p.vTx == VESC_TX_DOUBLE32:
            return decode_double32(data, offset, p.vTxDoubleScale)
        elif p.vTx == VESC_TX_DOUBLE32_AUTO:
            return decode_double32_auto(data, offset)
        else:
            raise ValueError(f"Param {p.name}: unsupported vTx={p.vTx} for DOUBLE")

    elif p.type == CFG_T_INT:
        if p.vTx == VESC_TX_UINT8:
            return decode_uint8(data, offset)
        elif p.vTx == VESC_TX_INT8:
            return decode_int8(data, offset)
        elif p.vTx == VESC_TX_UINT16:
            return decode_uint16(data, offset)
        elif p.vTx == VESC_TX_INT16:
            return decode_int16(data, offset)
        elif p.vTx == VESC_TX_UINT32:
            return decode_uint32(data, offset)
        elif p.vTx == VESC_TX_INT32:
            return decode_int32(data, offset)
        else:
            raise ValueError(f"Param {p.name}: unsupported vTx={p.vTx} for INT")

    elif p.type == CFG_T_QSTRING:
        val, new_offset = decode_string(data, offset)
        return val, new_offset

    elif p.type == CFG_T_INT and p.type == CFG_T_BITFIELD:
        # Bitfield uses uint8 in deserialization
        return decode_uint8(data, offset)

    elif p.type in (CFG_T_ENUM, CFG_T_BOOL):
        # Both use vbPopFrontInt8
        return decode_int8(data, offset)

    elif p.type == CFG_T_BITFIELD:
        # In setParamSerial, BITFIELD falls into CFG_T_INT branch but uses uint8
        return decode_uint8(data, offset)

    else:
        raise ValueError(f"Param {p.name}: unsupported type={p.type}")


def serialize_config(params, serialize_order, signature):
    """
    Serialize a full config: [signature (4B)] + [all params in order].
    Only params in serialize_order are included (they already exclude
    non-transmittable ones by design of the SerOrder in the XML).
    """
    data = encode_uint32(signature)
    for name in serialize_order:
        p = params.get(name)
        if p is None:
            raise ValueError(f"Parameter '{name}' in serialize order but not in params dict")
        data += serialize_param(p)
    return data


def deserialize_config(params, serialize_order, data, offset=0):
    """
    Deserialize config from data starting at offset.
    Returns dict of {param_name: value}.
    """
    values = {}
    for name in serialize_order:
        p = params.get(name)
        if p is None:
            raise ValueError(f"Parameter '{name}' in serialize order but not in params dict")
        val, offset = deserialize_param(p, data, offset)
        values[name] = val
    return values, offset


# ---------------------------------------------------------------------------
# Apply user config values onto param definitions
# ---------------------------------------------------------------------------

def apply_user_values(params, user_values):
    """
    Apply user config XML values onto the param definitions.
    Returns list of (name, old_val, new_val) for changed params.
    """
    changes = []
    for name, text in user_values.items():
        if name not in params:
            # Could be ConfigVersion or unknown param - skip silently
            continue
        p = params[name]
        if p.type in (CFG_T_BOOL, CFG_T_ENUM, CFG_T_INT, CFG_T_BITFIELD):
            new_val = int(text)
            if p.valInt != new_val:
                changes.append((name, p.valInt, new_val))
            p.valInt = new_val
        elif p.type == CFG_T_DOUBLE:
            new_val = float(text)
            if p.valDouble != new_val:
                changes.append((name, p.valDouble, new_val))
            p.valDouble = new_val
        elif p.type == CFG_T_QSTRING:
            new_val = text
            if p.valString != new_val:
                changes.append((name, p.valString, new_val))
            p.valString = new_val
    return changes


# ---------------------------------------------------------------------------
# VESC Packet framing - from packet.cpp
# ---------------------------------------------------------------------------

def frame_packet(payload: bytes) -> bytes:
    """
    Wrap payload in VESC packet format.
    Short packet (len <= 255): [0x02] [len 1B] [payload] [CRC16 2B] [0x03]
    Long packet (len <= 65535): [0x03] [len 2B] [payload] [CRC16 2B] [0x03]
    """
    length = len(payload)
    crc = crc16(payload)

    if length <= 255:
        frame = bytes([0x02, length & 0xFF])
    elif length <= 65535:
        frame = bytes([0x03, (length >> 8) & 0xFF, length & 0xFF])
    else:
        frame = bytes([0x04, (length >> 16) & 0xFF, (length >> 8) & 0xFF, length & 0xFF])

    frame += payload
    frame += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    frame += bytes([0x03])
    return frame


def parse_packet(data: bytes):
    """
    Try to parse a VESC packet from received data.
    Returns (payload, remaining_data) or (None, data) if incomplete.
    """
    while len(data) > 0:
        start_byte = data[0]

        if start_byte == 0x02:
            # Short packet
            if len(data) < 2:
                return None, data
            pkt_len = data[1]
            total_len = 2 + pkt_len + 3  # header + payload + crc(2) + end(1)
            if len(data) < total_len:
                return None, data
            payload = data[2:2 + pkt_len]
            crc_recv = (data[2 + pkt_len] << 8) | data[2 + pkt_len + 1]
            end_byte = data[2 + pkt_len + 2]
            if end_byte != 0x03:
                data = data[1:]
                continue
            crc_calc = crc16(payload)
            if crc_calc != crc_recv:
                data = data[1:]
                continue
            return payload, data[total_len:]

        elif start_byte == 0x03:
            # Long packet
            if len(data) < 3:
                return None, data
            pkt_len = (data[1] << 8) | data[2]
            total_len = 3 + pkt_len + 3
            if len(data) < total_len:
                return None, data
            payload = data[3:3 + pkt_len]
            crc_recv = (data[3 + pkt_len] << 8) | data[3 + pkt_len + 1]
            end_byte = data[3 + pkt_len + 2]
            if end_byte != 0x03:
                data = data[1:]
                continue
            crc_calc = crc16(payload)
            if crc_calc != crc_recv:
                data = data[1:]
                continue
            return payload, data[total_len:]

        elif start_byte == 0x04:
            # Extra-long packet
            if len(data) < 4:
                return None, data
            pkt_len = (data[1] << 16) | (data[2] << 8) | data[3]
            total_len = 4 + pkt_len + 3
            if len(data) < total_len:
                return None, data
            payload = data[4:4 + pkt_len]
            crc_recv = (data[4 + pkt_len] << 8) | data[4 + pkt_len + 1]
            end_byte = data[4 + pkt_len + 2]
            if end_byte != 0x03:
                data = data[1:]
                continue
            crc_calc = crc16(payload)
            if crc_calc != crc_recv:
                data = data[1:]
                continue
            return payload, data[total_len:]
        else:
            # Skip invalid byte
            data = data[1:]
            continue

    return None, data


# ---------------------------------------------------------------------------
# Serial communication
# ---------------------------------------------------------------------------

class VescSerial:
    """Handles serial communication with a VESC."""

    def __init__(self, port, baudrate=115200, timeout=2.0):
        import serial
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.port = port
        self.baudrate = baudrate
        # Flush any stale data
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_packet(self, payload: bytes):
        """Send a framed VESC packet."""
        frame = frame_packet(payload)
        self.ser.write(frame)
        self.ser.flush()

    def recv_packet(self, timeout=3.0):
        """
        Receive a single VESC packet. Returns the payload bytes or None on timeout.
        """
        buf = b''
        deadline = time.time() + timeout
        while time.time() < deadline:
            available = self.ser.in_waiting
            if available > 0:
                buf += self.ser.read(available)
                payload, remaining = parse_packet(buf)
                if payload is not None:
                    return payload
                buf = remaining
            else:
                time.sleep(0.01)
        return None

    def get_fw_version(self):
        """Request firmware version info."""
        self.send_packet(bytes([0]))  # COMM_FW_VERSION = 0
        resp = self.recv_packet()
        if resp is None:
            return None
        if resp[0] != 0:
            return None
        major = resp[1]
        minor = resp[2]
        # HW name follows after some fields
        info = {'major': major, 'minor': minor}
        # Parse the rest: fwName is a null-terminated string starting at offset 3
        offset = 3
        hw_end = resp.index(0, offset)
        info['hw'] = resp[offset:hw_end].decode('utf-8', errors='replace')
        return info

    def get_config(self, comm_get_cmd):
        """
        Request a config from VESC (COMM_GET_MCCONF or COMM_GET_APPCONF).
        Returns raw payload (including command byte) or None.
        """
        self.send_packet(bytes([comm_get_cmd]))
        # Config responses can be large, give extra time
        resp = self.recv_packet(timeout=5.0)
        if resp is None:
            return None
        if resp[0] != comm_get_cmd:
            print(f"  WARNING: Expected response cmd={comm_get_cmd}, got cmd={resp[0]}")
            return None
        return resp

    def set_config(self, comm_set_cmd, config_data: bytes):
        """
        Send a config to VESC. config_data is the serialized config
        (signature + params). Returns True if ACK received.
        """
        payload = bytes([comm_set_cmd]) + config_data
        self.send_packet(payload)
        # Wait for ACK (VESC echoes back the command byte)
        resp = self.recv_packet(timeout=5.0)
        if resp is None:
            return False
        if resp[0] == comm_set_cmd:
            return True
        print(f"  WARNING: Expected ACK cmd={comm_set_cmd}, got cmd={resp[0]}")
        return False


# ---------------------------------------------------------------------------
# High-level operations
# ---------------------------------------------------------------------------

# VESC firmware parameter definitions shipped next to this script
# (tools/vesc/params/6.02/, copied from vesc_tool res/config/6.02).
FW_PARAM_DIR = Path(__file__).resolve().parent / 'params' / '6.02'
MCCONF_PARAMS_XML = str(FW_PARAM_DIR / 'parameters_mcconf.xml')
APPCONF_PARAMS_XML = str(FW_PARAM_DIR / 'parameters_appconf.xml')

DEFAULT_PORT = '/dev/vesc'
DEFAULT_BAUD = 115200


def load_config(param_xml, user_xml, config_tag):
    """
    Load parameter definitions and user values, return everything needed
    for serialization.

    Returns (params, serialize_order, signature, changes).
    """
    params, serialize_order = parse_param_definitions_xml(param_xml)
    user_values = parse_user_config_xml(user_xml, config_tag)
    changes = apply_user_values(params, user_values)
    signature = calc_signature(params, serialize_order)
    return params, serialize_order, signature, changes


def print_config_summary(config_name, params, serialize_order, signature, changes):
    """Print a summary of the config to be uploaded."""
    print(f"\n{'='*60}")
    print(f"  {config_name} Configuration Summary")
    print(f"{'='*60}")
    print(f"  Signature: 0x{signature:08X}")
    print(f"  Total transmittable params: {len(serialize_order)}")
    print(f"  Params changed from defaults: {len(changes)}")
    if changes:
        print(f"\n  Changed parameters:")
        for name, old_val, new_val in changes[:30]:
            print(f"    {name}: {old_val} -> {new_val}")
        if len(changes) > 30:
            print(f"    ... and {len(changes) - 30} more")

    # Calculate serialized size
    config_data = serialize_config(params, serialize_order, signature)
    payload = bytes([0x00]) + config_data  # placeholder cmd byte
    print(f"  Serialized payload size: {len(payload)} bytes")
    print(f"{'='*60}")


def verify_config(vesc, params, serialize_order, signature, comm_get_cmd, config_name):
    """
    Read config back from VESC and compare to our params.
    Returns True if matching, False otherwise.
    """
    print(f"\n  Verifying {config_name} config...")
    resp = vesc.get_config(comm_get_cmd)
    if resp is None:
        print(f"  ERROR: Failed to read {config_name} config from VESC")
        return False

    # resp[0] is the command byte, then comes the config data
    data = resp[1:]

    # Check signature
    recv_sig = struct.unpack('>I', data[0:4])[0]
    if recv_sig != signature:
        print(f"  ERROR: Signature mismatch! Expected 0x{signature:08X}, got 0x{recv_sig:08X}")
        return False

    print(f"  Signature matches: 0x{signature:08X}")

    # Deserialize and compare
    recv_values, _ = deserialize_config(params, serialize_order, data, offset=4)

    mismatches = []
    for name in serialize_order:
        p = params[name]
        recv_val = recv_values[name]

        if p.type == CFG_T_DOUBLE:
            expected = p.valDouble
            # Allow some tolerance for encoding precision
            eps = 0.01 if p.vTx == VESC_TX_DOUBLE16 else 0.001
            if p.vTx == VESC_TX_DOUBLE32_AUTO:
                # DOUBLE32_AUTO uses float precision
                eps = max(abs(expected) * 1e-6, 1e-6)
            if abs(recv_val - expected) > eps:
                mismatches.append((name, expected, recv_val))
        elif p.type in (CFG_T_INT, CFG_T_BOOL, CFG_T_ENUM, CFG_T_BITFIELD):
            if int(recv_val) != p.valInt:
                mismatches.append((name, p.valInt, recv_val))
        elif p.type == CFG_T_QSTRING:
            if recv_val != p.valString:
                mismatches.append((name, p.valString, recv_val))

    if mismatches:
        print(f"  VERIFICATION FAILED - {len(mismatches)} mismatched params:")
        for name, expected, got in mismatches[:20]:
            print(f"    {name}: expected={expected}, got={got}")
        if len(mismatches) > 20:
            print(f"    ... and {len(mismatches) - 20} more")
        return False
    else:
        print(f"  VERIFICATION OK - all {len(serialize_order)} params match")
        return True


def do_upload(vesc, params, serialize_order, signature, comm_set_cmd, comm_get_cmd,
              config_name, dry_run=False, verify=False):
    """Upload a config to VESC."""
    config_data = serialize_config(params, serialize_order, signature)

    if dry_run:
        print(f"\n  [DRY RUN] Would send {config_name} config:")
        print(f"    Command: 0x{comm_set_cmd:02X}")
        print(f"    Signature: 0x{signature:08X}")
        print(f"    Payload size: {len(config_data) + 1} bytes")
        print(f"    First 32 bytes of serialized data: {config_data[:32].hex()}")
        return True

    print(f"\n  Uploading {config_name} config...")
    ok = vesc.set_config(comm_set_cmd, config_data)
    if ok:
        print(f"  {config_name} config written successfully!")
    else:
        print(f"  ERROR: Failed to write {config_name} config!")
        return False

    if verify:
        # Wait a moment for VESC to process
        time.sleep(0.5)
        return verify_config(vesc, params, serialize_order, signature,
                             comm_get_cmd, config_name)

    return True


def do_signature_check(vesc, params, serialize_order, signature, comm_get_cmd, config_name):
    """
    Read the current config from VESC and check if our computed signature
    matches the one the VESC sends. This validates our parameter definitions
    are correct for the firmware version.
    """
    print(f"\n  Checking {config_name} signature against VESC...")
    resp = vesc.get_config(comm_get_cmd)
    if resp is None:
        print(f"  ERROR: Failed to read {config_name} config from VESC")
        return False

    data = resp[1:]
    recv_sig = struct.unpack('>I', data[0:4])[0]

    if recv_sig == signature:
        print(f"  Signature MATCH: 0x{signature:08X}")
        return True
    else:
        print(f"  Signature MISMATCH!")
        print(f"    Computed: 0x{signature:08X}")
        print(f"    From VESC: 0x{recv_sig:08X}")
        print(f"  This means the parameter definitions don't match the firmware.")
        return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Upload motor and/or app configuration to a VESC via serial.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # upload the car's stored motor + app config and read it back
  %(prog)s --motor tools/vesc/configs/motor_config.xml \
           --app tools/vesc/configs/app_config.xml --verify
  %(prog)s --motor tools/vesc/configs/motor_config.xml
  %(prog)s --app tools/vesc/configs/app_config.xml
  %(prog)s --motor tools/vesc/configs/motor_config.xml --dry-run
  %(prog)s --check-sig
        """)

    parser.add_argument('--motor', metavar='FILE',
                        help='Motor config XML file to upload')
    parser.add_argument('--app', metavar='FILE',
                        help='App config XML file to upload')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Show what would be sent without actually sending')
    parser.add_argument('--verify', action='store_true',
                        help='Read back config after writing and verify')
    parser.add_argument('--check-sig', action='store_true',
                        help='Only check signature against VESC (no upload)')
    parser.add_argument('--mcconf-params', default=MCCONF_PARAMS_XML,
                        help='Motor config parameter definitions XML '
                             '(default: %(default)s)')
    parser.add_argument('--appconf-params', default=APPCONF_PARAMS_XML,
                        help='App config parameter definitions XML '
                             '(default: %(default)s)')

    args = parser.parse_args()

    if not args.motor and not args.app and not args.check_sig:
        parser.error('At least one of --motor, --app, or --check-sig is required')

    # Load motor config if requested
    mc_data = None
    if args.motor:
        print(f"Loading motor config from: {args.motor}")
        mc_params, mc_order, mc_sig, mc_changes = load_config(
            args.mcconf_params, args.motor, 'MCConfiguration')
        print_config_summary('Motor', mc_params, mc_order, mc_sig, mc_changes)
        mc_data = (mc_params, mc_order, mc_sig)

    # Load app config if requested
    app_data = None
    if args.app:
        print(f"Loading app config from: {args.app}")
        app_params, app_order, app_sig, app_changes = load_config(
            args.appconf_params, args.app, 'APPConfiguration')
        print_config_summary('App', app_params, app_order, app_sig, app_changes)
        app_data = (app_params, app_order, app_sig)

    # Check-sig only mode
    if args.check_sig:
        if args.motor:
            mc_params_check, mc_order_check, mc_sig_check, _ = load_config(
                args.mcconf_params, args.motor, 'MCConfiguration')
        else:
            mc_params_check, mc_order_check = parse_param_definitions_xml(args.mcconf_params)
            mc_sig_check = calc_signature(mc_params_check, mc_order_check)

        if args.app:
            app_params_check, app_order_check, app_sig_check, _ = load_config(
                args.appconf_params, args.app, 'APPConfiguration')
        else:
            app_params_check, app_order_check = parse_param_definitions_xml(args.appconf_params)
            app_sig_check = calc_signature(app_params_check, app_order_check)

        print(f"\nComputed signatures:")
        print(f"  Motor config: 0x{mc_sig_check:08X}")
        print(f"  App config:   0x{app_sig_check:08X}")

    # If dry-run, show what we'd send and exit
    if args.dry_run:
        if mc_data:
            mc_params, mc_order, mc_sig = mc_data
            do_upload(None, mc_params, mc_order, mc_sig,
                      COMM_SET_MCCONF, COMM_GET_MCCONF, 'Motor',
                      dry_run=True)
        if app_data:
            app_params, app_order, app_sig = app_data
            do_upload(None, app_params, app_order, app_sig,
                      COMM_SET_APPCONF, COMM_GET_APPCONF, 'App',
                      dry_run=True)
        return

    # Open serial connection
    if not args.dry_run:
        try:
            import serial
        except ImportError:
            print("ERROR: pyserial is required. Install with: pip install pyserial")
            sys.exit(1)

    print(f"\nConnecting to VESC at {args.port} ({args.baud} baud)...")
    try:
        vesc = VescSerial(args.port, args.baud)
    except Exception as e:
        print(f"ERROR: Failed to open serial port: {e}")
        sys.exit(1)

    try:
        # Get firmware version
        fw = vesc.get_fw_version()
        if fw:
            print(f"  Firmware: {fw['major']}.{fw['minor']:02d}, HW: {fw.get('hw', 'unknown')}")
        else:
            print("  WARNING: Could not read firmware version")

        # Signature check mode
        if args.check_sig:
            print()
            sig_ok = True
            sig_ok &= do_signature_check(vesc, mc_params_check, mc_order_check,
                                         mc_sig_check, COMM_GET_MCCONF, 'Motor')
            sig_ok &= do_signature_check(vesc, app_params_check, app_order_check,
                                         app_sig_check, COMM_GET_APPCONF, 'App')
            if not sig_ok:
                print("\nSignature check FAILED. Parameter definitions may not match firmware.")
                sys.exit(1)
            else:
                print("\nAll signatures match!")
            if not mc_data and not app_data:
                return

        # Confirm before uploading
        if mc_data or app_data:
            configs_to_upload = []
            if mc_data:
                configs_to_upload.append('Motor')
            if app_data:
                configs_to_upload.append('App')

            print(f"\n>>> Ready to upload: {', '.join(configs_to_upload)} config(s)")
            print(f">>> Target: {args.port}")
            if fw:
                print(f">>> Firmware: {fw['major']}.{fw['minor']:02d}")
            response = input(">>> Type 'yes' to proceed, anything else to abort: ")
            if response.strip().lower() != 'yes':
                print("Aborted.")
                return

        # Upload motor config
        if mc_data:
            mc_params, mc_order, mc_sig = mc_data
            ok = do_upload(vesc, mc_params, mc_order, mc_sig,
                           COMM_SET_MCCONF, COMM_GET_MCCONF, 'Motor',
                           verify=args.verify)
            if not ok:
                print("Motor config upload failed!")
                sys.exit(1)

        # Upload app config
        if app_data:
            app_params, app_order, app_sig = app_data
            # Small delay between configs
            if mc_data:
                time.sleep(1.0)
            ok = do_upload(vesc, app_params, app_order, app_sig,
                           COMM_SET_APPCONF, COMM_GET_APPCONF, 'App',
                           verify=args.verify)
            if not ok:
                print("App config upload failed!")
                sys.exit(1)

        print("\nDone!")

    finally:
        vesc.close()


if __name__ == '__main__':
    main()
