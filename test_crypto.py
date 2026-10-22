#!/usr/bin/env python3
"""
Brgy. San Miguel Emergency LoRa Mesh - packet format and crypto check.

Mirrors the firmware (include/packet_codec.h + include/aes_crypto.h):
  DATA frame  = 10-byte prefix + [16-byte IV + AES-128-CBC ciphertext] + 16-byte frame tag
                (prefix bytes 8-9 = relay path: up to 5 relays, 3 bits each)
  ACK frame   = 8-byte prefix + 16-byte frame tag (24 bytes)
  HEARTBEAT   = 21-byte body (incl. own + one relayed link-state row) + 8-byte tag (29 bytes)
  frame tag   = HMAC-SHA-256(mac_key, everything before the tag), truncated
  body        = ts (uint32 BE, Unix s) | ms (uint16 BE, 0-999) | location length (1 byte) | location | message
  location + message <= 200 bytes, so a frame never exceeds the 250-byte LoRa MTU.

Usage:
  python test_crypto.py              run the self-test
  python test_crypto.py <frame-hex>  decode a DATA frame captured from a node
"""

import hashlib
import hmac
import os
from pathlib import Path
import re
import struct
import sys

from cryptography.hazmat.primitives import padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

# Read the same local-only key used by the firmware so no copy stays in the repo.
def load_pre_shared_key() -> bytes:
    config = Path(__file__).resolve().parent / "include" / "private_config.h"
    if not config.exists():
        raise RuntimeError("Missing include/private_config.h; copy private_config.example.h and add local secrets.")
    source = config.read_text(encoding="utf-8")
    match = re.search(r"PRE_SHARED_KEY\s*\[\s*16\s*\]\s*=\s*\{([^}]*)\}", source, re.S)
    values = re.findall(r"0x([0-9A-Fa-f]{2})", match.group(1)) if match else []
    if len(values) != 16:
        raise RuntimeError("PRE_SHARED_KEY in include/private_config.h must contain exactly 16 hex bytes.")
    return bytes(int(value, 16) for value in values)


PRE_SHARED_KEY = load_pre_shared_key()
MAC_KEY_LABEL = b"SanMiguel LoRa CBC MAC v1"

MAX_LORA_PACKET_SIZE = 250
FRAME_PREFIX_SIZE = 10
ACK_PREFIX_SIZE = 8
FRAME_TAG_SIZE = 16
HEARTBEAT_BODY_SIZE = 21
HEARTBEAT_TAG_SIZE = 8
MAX_TEXT_BYTES = 200

TYPE_SOS, TYPE_HEARTBEAT, TYPE_TELEMETRY, TYPE_ACK = 0, 1, 2, 3


def mac_key() -> bytes:
    return hmac.new(PRE_SHARED_KEY, MAC_KEY_LABEL, hashlib.sha256).digest()


def tag(data: bytes, length: int) -> bytes:
    return hmac.new(mac_key(), data, hashlib.sha256).digest()[:length]


def header(k: int, ttl: int, ptype: int) -> int:
    return ((k & 0x03) << 6) | ((ttl & 0x0F) << 2) | (ptype & 0x03)


def build_body(ts: int, location: str, message: str, ms: int = 0) -> bytes:
    loc, msg = location.encode("utf-8"), message.encode("utf-8")
    if len(loc) > 255 or len(loc) + len(msg) > MAX_TEXT_BYTES:
        raise ValueError(f"location + message is {len(loc) + len(msg)} bytes (max {MAX_TEXT_BYTES})")
    return struct.pack(">IHB", ts, ms, len(loc)) + loc + msg


def parse_body(body: bytes):
    ts, ms, loc_len = struct.unpack(">IHB", body[:7])
    if ms > 999:
        raise ValueError("milliseconds out of range")
    loc = body[7:7 + loc_len]
    if len(loc) != loc_len:
        raise ValueError("location length exceeds body")
    return ts, ms, loc.decode("utf-8"), body[7 + loc_len:].decode("utf-8")


def encrypt_body(plain: bytes) -> bytes:
    iv = os.urandom(16)
    padder = padding.PKCS7(128).padder()
    padded = padder.update(plain) + padder.finalize()
    enc = Cipher(algorithms.AES(PRE_SHARED_KEY), modes.CBC(iv)).encryptor()
    return iv + enc.update(padded) + enc.finalize()


def decrypt_body(blob: bytes) -> bytes:
    iv, ct = blob[:16], blob[16:]
    if not ct or len(ct) % 16:
        raise ValueError("invalid ciphertext length")
    dec = Cipher(algorithms.AES(PRE_SHARED_KEY), modes.CBC(iv)).decryptor()
    padded = dec.update(ct) + dec.finalize()
    unpadder = padding.PKCS7(128).unpadder()
    return unpadder.update(padded) + unpadder.finalize()


def relay_path(*relays):
    path = 0
    for slot, node in enumerate(relays[:5]):
        path |= (node & 0x07) << (3 * slot)
    return path


def decode_relay_path(path):
    relays = []
    for slot in range(5):
        node = (path >> (3 * slot)) & 0x07
        if not node:
            break
        relays.append(node)
    return relays


def build_data_frame(k, ttl, ptype, src, next_hop, dst, origin, msg_id, ts, location, message,
                     mac_retries=0, relays=(), ms=0):
    path = relay_path(*relays)
    prefix = bytes([header(k, ttl, ptype), src, next_hop, dst, origin, msg_id >> 8, msg_id & 0xFF,
                    mac_retries, path >> 8, path & 0xFF])
    unsigned = prefix + encrypt_body(build_body(ts, location, message, ms))
    frame = unsigned + tag(unsigned, FRAME_TAG_SIZE)
    if len(frame) > MAX_LORA_PACKET_SIZE:
        raise ValueError("frame exceeds 250-byte LoRa MTU")
    return frame


def decode_data_frame(frame: bytes) -> dict:
    if not FRAME_PREFIX_SIZE + FRAME_TAG_SIZE < len(frame) <= MAX_LORA_PACKET_SIZE:
        raise ValueError("invalid frame length")
    unsigned, received = frame[:-FRAME_TAG_SIZE], frame[-FRAME_TAG_SIZE:]
    if not hmac.compare_digest(received, tag(unsigned, FRAME_TAG_SIZE)):
        raise ValueError("frame tag mismatch: altered packet or wrong key")
    h = frame[0]
    ts, ms, location, message = parse_body(decrypt_body(unsigned[FRAME_PREFIX_SIZE:]))
    return {
        "k": h >> 6, "ttl": (h >> 2) & 0x0F, "type": h & 0x03,
        "source": frame[1], "next_hop": frame[2], "destination": frame[3], "origin": frame[4],
        "id": (frame[5] << 8) | frame[6], "mac_retries": frame[7],
        "relays": decode_relay_path((frame[8] << 8) | frame[9]),
        "ts": ts, "ms": ms, "location": location, "message": message, "frame_bytes": len(frame),
    }


def decode_relay_path_of(frame):
    return decode_relay_path((frame[8] << 8) | frame[9])


def build_heartbeat(src, seq, battery_permille, traffic, own_row=(255,) * 5,
                    relayed_owner=0, relayed_seq=0, relayed_row=(255,) * 5):
    """Byte 7 is reserved zero; rows contain 5 weights (255 = no edge)."""
    body = (bytes([header(0, 0, TYPE_HEARTBEAT), src]) + struct.pack(">HHBB", seq, battery_permille, traffic, 0)
            + bytes(own_row) + bytes([relayed_owner]) + struct.pack(">H", relayed_seq) + bytes(relayed_row))
    return body + tag(body, HEARTBEAT_TAG_SIZE)


def time_on_air_s(n_bytes, sf=12, bw=125e3, cr=1, preamble=8):
    """Semtech LoRa time-on-air (explicit header, CRC on, LDRO on at SF11/12 BW125)."""
    t_sym = (2 ** sf) / bw
    de = 1 if t_sym > 0.016 else 0
    n_payload = 8 + max(-(-(8 * n_bytes - 4 * sf + 28 + 16) // (4 * (sf - 2 * de))) * (cr + 4), 0)
    return (preamble + 4.25 + n_payload) * t_sym


def self_test():
    print("=" * 65)
    print("  BRGY. SAN MIGUEL LORA MESH - PACKET FORMAT & SECURITY TEST")
    print("=" * 65)

    worst = build_data_frame(1, 15, TYPE_SOS, 6, 5, 1, 6, 1042, 1791796202, "", "A" * 200)
    print(f"[1] 200-byte message            -> {len(worst)} bytes (MTU 250), ToA {time_on_air_s(len(worst)):.2f} s")
    assert len(worst) <= MAX_LORA_PACKET_SIZE and decode_data_frame(worst)["message"] == "A" * 200

    mixed = build_data_frame(2, 15, TYPE_TELEMETRY, 6, 5, 1, 6, 1043, 1791796202, "L" * 80, "M" * 120,
                             relays=(3, 5, 2, 4, 6))
    print(f"[2] 80 B location + 120 B text, 5 relays -> {len(mixed)} bytes, relays {decode_relay_path_of(mixed)}")
    assert len(mixed) <= MAX_LORA_PACKET_SIZE and decode_data_frame(mixed)["relays"] == [3, 5, 2, 4, 6]

    try:
        build_data_frame(1, 15, TYPE_SOS, 6, 5, 1, 6, 1, 1, "X", "A" * 200)
        raise AssertionError("201 bytes should be rejected")
    except ValueError:
        print("[3] 201 bytes of text           -> rejected")

    sample = build_data_frame(1, 15, TYPE_SOS, 6, 5, 1, 6, 1044, 1791796202,
                              "Purok 4, Coastline",
                              "Elderly resident with acute respiratory distress. Water level rising fast.",
                              ms=437)
    d = decode_data_frame(sample)
    assert d["ts"] == 1791796202 and d["ms"] == 437
    print(f"[4] Typical SOS                 -> {len(sample)} bytes, ToA {time_on_air_s(len(sample)):.2f} s; "
          f"decoded V{d['origin']}#{d['id']}: {d['message'][:30]}...")

    tampered = bytearray(sample)
    tampered[20] ^= 0x01
    try:
        decode_data_frame(bytes(tampered))
        raise AssertionError("tampered frame accepted")
    except ValueError:
        print("[5] Tampered frame              -> rejected by frame tag")

    hb = build_heartbeat(5, 77, 0xFFFF, 4, own_row=(255, 6, 12, 255, 2), relayed_owner=3, relayed_seq=40,
                         relayed_row=(4, 255, 255, 12, 15))
    print(f"[6] Heartbeat                   -> {len(hb)} bytes, ToA {time_on_air_s(len(hb)):.2f} s")
    print(f"[7] ACK                         -> 24 bytes, ToA {time_on_air_s(24):.2f} s")
    assert len(hb) == 29 and hb[7] == 0

    print("\n[+] SUCCESS: binary frames, AES-128-CBC and HMAC-SHA-256 frame tags verified.")
    print("=" * 65)


if __name__ == "__main__":
    if len(sys.argv) == 2:
        print(decode_data_frame(bytes.fromhex(sys.argv[1])))
    else:
        self_test()
