import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

import numpy as np
import cv2
import subprocess
import time
import os
import asyncio
import socket
from datetime import datetime
from collections import deque
from bleak import BleakClient


# ============================================================
# BLE ACK setting
# ============================================================
# Your ESP32_S3_TX MAC address
ESP32_ADDRESS = "20:6E:F1:B2:BF:15"

# ESP32-S3 notifies prompt status on TX; Jetson writes prompt/packet ACKs to RX.
TX_CHAR_UUID = "12345678-1234-1234-1234-1234567890ac"
RX_CHAR_UUID = "12345678-1234-1234-1234-1234567890ad"
PROMPT_ACK_TIMEOUT_SEC = 15
PC_BLUETOOTH_ADDRESS = "80:45:DD:05:F3:65"
PC_RFCOMM_CHANNEL = 4
PC_BLUETOOTH_TIMEOUT_SEC = 930


# ============================================================
# Save setting
# ============================================================
SAVE_DIR = "/home/jetson/capture_output"

RUN_ID = datetime.now().strftime("realtime_ble_ack_run_%Y%m%d_%H%M%S")
RUN_DIR = os.path.join(SAVE_DIR, RUN_ID)

os.makedirs(RUN_DIR, exist_ok=True)


# ============================================================
# Camera setting
# ============================================================
SENSOR_ID = 0

CAP_WIDTH = 1280
CAP_HEIGHT = 720

FPS = 60

SHUTTER_SPEED = 1 / 600
EXPOSURE_NS = int(SHUTTER_SPEED * 1e9)

ANALOG_GAIN = 4
ISP_DIGITAL_GAIN = 1


# ============================================================
# ROI / brightness setting
# ============================================================
USE_CENTER_ROI = True
ROI_RATIO = 0.05

MEAN_THRESHOLD = 150
BRIGHT_PIXEL_THRESHOLD = 180
BRIGHT_RATIO_THRESHOLD = 0.4


# ============================================================
# Protocol setting
# ============================================================
PREAMBLE_BITS = "1111100110101"
PREAMBLE_LEN = 13

HEADER_BYTES = 1
PAYLOAD_BYTES = 8
CRC_BYTES = 1

RAW_BYTES = HEADER_BYTES + PAYLOAD_BYTES + CRC_BYTES
RAW_BITS = RAW_BYTES * 8

NIBBLE_COUNT = RAW_BITS // 4
HAMMING_BITS_PER_CODEWORD = 7

ENCODED_BITS = NIBBLE_COUNT * HAMMING_BITS_PER_CODEWORD
TX_BITS_TOTAL = PREAMBLE_LEN + ENCODED_BITS

MAX_PREAMBLE_ERROR = 0


# ============================================================
# Realtime state
# ============================================================
STATE_SEARCH_PREAMBLE = "SEARCH_PREAMBLE"
STATE_RECEIVE_ENCODED = "RECEIVE_ENCODED"

rx_state = STATE_SEARCH_PREAMBLE

preamble_buffer = deque(maxlen=PREAMBLE_LEN)
encoded_buffer = []

raw_bit_log = ""

detected_preamble_count = 0
crc_fail_count = 0
frame_count = 0

decoded_done = False


# ============================================================
# Multi-packet receive state
# ============================================================
received_packets = {}
received_packet_order = []
full_payload_bytes = bytearray()
last_packet_received = False


# ============================================================
# Camera helper
# ============================================================
def reset_camera():
    print("Resetting nvargus-daemon...")
    subprocess.run(
        ["sudo", "systemctl", "restart", "nvargus-daemon"],
        check=False
    )
    time.sleep(1.0)


def create_pipeline():
    return (
        f"nvarguscamerasrc sensor-id={SENSOR_ID} "
        f"exposuretimerange='{EXPOSURE_NS} {EXPOSURE_NS}' "
        f"gainrange='{ANALOG_GAIN} {ANALOG_GAIN}' "
        f"ispdigitalgainrange='{ISP_DIGITAL_GAIN} {ISP_DIGITAL_GAIN}' "
        f"aeantibanding=0 ! "
        f"video/x-raw(memory:NVMM), "
        f"width={CAP_WIDTH}, height={CAP_HEIGHT}, "
        f"format=NV12, framerate={FPS}/1 ! "
        f"nvvidconv ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
    )


# ============================================================
# ROI helper
# ============================================================
def get_roi_box(width, height):
    if USE_CENTER_ROI:
        roi_w = int(width * ROI_RATIO)
        roi_h = int(height * ROI_RATIO)

        x1 = (width - roi_w) // 2
        y1 = (height - roi_h) // 2
        x2 = x1 + roi_w
        y2 = y1 + roi_h
    else:
        x1 = 0
        y1 = 0
        x2 = width
        y2 = height

    return x1, y1, x2, y2


def analyze_led_state(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    h, w = gray.shape
    x1, y1, x2, y2 = get_roi_box(w, h)

    roi = gray[y1:y2, x1:x2]

    mean_gray = roi.mean()
    bright_ratio = np.mean(roi > BRIGHT_PIXEL_THRESHOLD)

    if bright_ratio > BRIGHT_RATIO_THRESHOLD or mean_gray >= MEAN_THRESHOLD:
        bit = "1"
    else:
        bit = "0"

    return bit, mean_gray, bright_ratio, (x1, y1, x2, y2)


def draw_roi_overlay(frame, bit, mean_gray, bright_ratio, roi_box, state_text):
    display = frame.copy()

    x1, y1, x2, y2 = roi_box

    if bit == "1":
        box_color = (0, 255, 0)
        bit_color = (0, 255, 0)
        bit_text = "LED = 1"
    else:
        box_color = (0, 0, 255)
        bit_color = (0, 0, 255)
        bit_text = "LED = 0"

    cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 3)

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    cv2.drawMarker(
        display,
        (cx, cy),
        box_color,
        markerType=cv2.MARKER_CROSS,
        markerSize=25,
        thickness=2
    )

    cv2.putText(
        display,
        bit_text,
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        bit_color,
        3
    )

    cv2.putText(
        display,
        f"STATE = {state_text}",
        (30, 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"ROI mean = {mean_gray:.2f}",
        (30, 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"bright_ratio = {bright_ratio:.5f}",
        (30, 165),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"preamble_count={detected_preamble_count} | crc_fail={crc_fail_count}",
        (30, 200),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"encoded_buffer={len(encoded_buffer)}/{ENCODED_BITS}",
        (30, 235),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"received_packets={len(received_packets)} | last={last_packet_received}",
        (30, 270),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"SAVE_DIR={SAVE_DIR}",
        (30, 305),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    return display


# ============================================================
# Basic helper
# ============================================================
def hamming_distance(a, b):
    return sum(x != y for x, y in zip(a, b))


def check_preamble(buffer):
    if len(buffer) < PREAMBLE_LEN:
        return False, None

    window = "".join(buffer)
    dist = hamming_distance(window, PREAMBLE_BITS)

    if dist <= MAX_PREAMBLE_ERROR:
        return True, dist

    return False, dist


# ============================================================
# Deinterleave
# ============================================================
def deinterleave_encoded_bits(encoded_bits):
    if len(encoded_bits) != ENCODED_BITS:
        raise ValueError(f"Encoded length must be {ENCODED_BITS}, got {len(encoded_bits)}")

    encoded = [["0"] * HAMMING_BITS_PER_CODEWORD for _ in range(NIBBLE_COUNT)]

    idx = 0

    for col in range(HAMMING_BITS_PER_CODEWORD):
        for row in range(NIBBLE_COUNT):
            encoded[row][col] = encoded_bits[idx]
            idx += 1

    codewords = ["".join(encoded[row]) for row in range(NIBBLE_COUNT)]

    return codewords


# ============================================================
# Hamming(7,4) decode with detailed report
# ============================================================
def hamming74_decode(codeword):
    if len(codeword) != 7:
        raise ValueError("Codeword length must be 7")

    original_bits = [int(b) for b in codeword]
    bits = original_bits.copy()

    d1, d2, d3, d4, p1, p2, p3 = bits

    s1 = d1 ^ d2 ^ d4 ^ p1
    s2 = d1 ^ d3 ^ d4 ^ p2
    s3 = d2 ^ d3 ^ d4 ^ p3

    syndrome = (s1, s2, s3)
    syndrome_str = f"{s1}{s2}{s3}"

    syndrome_to_bit_index = {
        (1, 1, 0): 0,  # d1
        (1, 0, 1): 1,  # d2
        (0, 1, 1): 2,  # d3
        (1, 1, 1): 3,  # d4
        (1, 0, 0): 4,  # p1
        (0, 1, 0): 5,  # p2
        (0, 0, 1): 6,  # p3
    }

    bit_names = ["d1", "d2", "d3", "d4", "p1", "p2", "p3"]

    corrected = False
    error_pos = None
    error_bit_name = "none"

    if syndrome != (0, 0, 0):
        if syndrome in syndrome_to_bit_index:
            error_pos = syndrome_to_bit_index[syndrome]
            error_bit_name = bit_names[error_pos]
            bits[error_pos] ^= 1
            corrected = True
        else:
            error_bit_name = "unknown"

    corrected_codeword = "".join(str(b) for b in bits)

    decoded_nibble_bits = bits[0:4]
    decoded_nibble_str = "".join(str(b) for b in decoded_nibble_bits)

    nibble_value = (
        (decoded_nibble_bits[0] << 3) |
        (decoded_nibble_bits[1] << 2) |
        (decoded_nibble_bits[2] << 1) |
        (decoded_nibble_bits[3] << 0)
    )

    detail = {
        "original_codeword": codeword,
        "corrected_codeword": corrected_codeword,
        "syndrome": syndrome,
        "syndrome_str": syndrome_str,
        "corrected": corrected,
        "error_pos": error_pos,
        "error_bit_name": error_bit_name,
        "decoded_nibble_bits": decoded_nibble_str,
        "nibble_value": nibble_value,
    }

    return nibble_value, detail


def decode_codewords_to_raw_bytes(codewords):
    if len(codewords) != NIBBLE_COUNT:
        raise ValueError(f"Expected {NIBBLE_COUNT} codewords, got {len(codewords)}")

    nibbles = []
    decode_report = []
    corrected_count = 0

    for i, cw in enumerate(codewords):
        nibble, detail = hamming74_decode(cw)

        if detail["corrected"]:
            corrected_count += 1

        nibbles.append(nibble)

        byte_index = i // 2
        nibble_position = "high" if i % 2 == 0 else "low"

        detail["index"] = i
        detail["byte_index"] = byte_index
        detail["nibble_position"] = nibble_position

        decode_report.append(detail)

    raw_bytes = []

    for i in range(0, len(nibbles), 2):
        high = nibbles[i]
        low = nibbles[i + 1]

        byte_value = (high << 4) | low
        raw_bytes.append(byte_value)

    for item in decode_report:
        byte_index = item["byte_index"]

        if byte_index < len(raw_bytes):
            item["byte_value"] = raw_bytes[byte_index]
        else:
            item["byte_value"] = None

    return raw_bytes, decode_report, corrected_count


# ============================================================
# CRC-8
# ============================================================
def crc8(data):
    crc = 0x00
    poly = 0x07

    for value in data:
        crc ^= value

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF

    return crc & 0xFF


# ============================================================
# Header / payload
# ============================================================
def parse_header(header):
    packet_id = (header >> 4) & 0x0F
    last_packet = ((header >> 3) & 0x01) == 1
    len_field = header & 0x07

    # Current design:
    # Payload is always fixed 8 bytes
    payload_len = 8

    return packet_id, last_packet, len_field, payload_len


def bytes_to_payload_text(payload_bytes):
    payload_8bytes = bytes(payload_bytes[:PAYLOAD_BYTES])

    useful_payload = payload_8bytes.rstrip(b"\x00")

    try:
        text = useful_payload.decode("ascii", errors="replace")
    except Exception:
        text = ""

    return text, useful_payload


# ============================================================
# Decode full packet
# ============================================================
def decode_encoded_bits(encoded_bits):
    codewords = deinterleave_encoded_bits(encoded_bits)

    raw_bytes, decode_report, corrected_count = decode_codewords_to_raw_bytes(codewords)

    if len(raw_bytes) != RAW_BYTES:
        return None

    header = raw_bytes[0]
    payload = raw_bytes[1:1 + PAYLOAD_BYTES]
    received_crc = raw_bytes[1 + PAYLOAD_BYTES]

    calculated_crc = crc8(raw_bytes[0:1 + PAYLOAD_BYTES])

    crc_ok = received_crc == calculated_crc

    packet_id, last_packet, len_field, payload_len = parse_header(header)

    text, useful_payload = bytes_to_payload_text(payload)

    result = {
        "codewords": codewords,
        "raw_bytes": raw_bytes,
        "decode_report": decode_report,
        "corrected_count": corrected_count,
        "header": header,
        "payload": payload,
        "received_crc": received_crc,
        "calculated_crc": calculated_crc,
        "crc_ok": crc_ok,
        "packet_id": packet_id,
        "last_packet": last_packet,
        "len_field": len_field,
        "payload_len": payload_len,
        "text": text,
        "useful_payload": useful_payload,
    }

    return result


# ============================================================
# Save helpers
# ============================================================
def save_raw_bit_log():
    path = os.path.join(RUN_DIR, "realtime_raw_bit_log.txt")

    with open(path, "w", encoding="utf-8") as f:
        f.write(raw_bit_log)

    print(f"[SAVE] Raw realtime bit log saved to: {path}")


def print_decode_result(result):
    print("\n========== Packet Decode Result ==========")
    print(f"Corrected Hamming codewords: {result['corrected_count']}")
    print(f"Raw 10 bytes hex: {bytes(result['raw_bytes']).hex(' ')}")
    print(f"Header: 0x{result['header']:02X}")
    print(f"Packet ID: {result['packet_id']}")
    print(f"Last packet: {result['last_packet']}")
    print(f"Header len field: {result['len_field']}  (0 means 8 bytes)")
    print(f"Payload fixed length: {result['payload_len']}")
    print(f"Payload 8 bytes hex: {bytes(result['payload']).hex(' ')}")
    print(f"Useful payload hex after stripping 0x00: {result['useful_payload'].hex(' ')}")
    print(f"Received CRC: 0x{result['received_crc']:02X}")
    print(f"Calculated CRC: 0x{result['calculated_crc']:02X}")
    print(f"CRC OK: {result['crc_ok']}")
    print(f"Decoded text: {result['text']}")
    print("==========================================\n")


def print_hamming_decode_report(result):
    print("\n========== Detailed Hamming Decode Report ==========")
    print("index | byte | pos  | original | syndrome | corrected | error | fixed   | nibble | hex | byte")
    print("-------------------------------------------------------------------------------------")

    for item in result["decode_report"]:
        index = item["index"]
        byte_index = item["byte_index"]
        nibble_position = item["nibble_position"]

        original_codeword = item["original_codeword"]
        syndrome_str = item["syndrome_str"]
        corrected = item["corrected"]
        error_bit_name = item["error_bit_name"]
        corrected_codeword = item["corrected_codeword"]
        decoded_nibble_bits = item["decoded_nibble_bits"]
        nibble_value = item["nibble_value"]
        byte_value = item["byte_value"]

        byte_hex = "--" if byte_value is None else f"0x{byte_value:02X}"

        print(
            f"{index:02d}    | "
            f"{byte_index:02d}   | "
            f"{nibble_position:4s} | "
            f"{original_codeword} | "
            f"{syndrome_str}      | "
            f"{str(corrected):9s} | "
            f"{error_bit_name:5s} | "
            f"{corrected_codeword} | "
            f"{decoded_nibble_bits}   | "
            f"0x{nibble_value:X} | "
            f"{byte_hex}"
        )

    print("====================================================\n")


def save_decode_result(result, filename):
    path = os.path.join(RUN_DIR, filename)

    with open(path, "w", encoding="utf-8") as f:
        f.write("========== Packet Decode Result ==========\n")
        f.write(f"Corrected Hamming codewords: {result['corrected_count']}\n")
        f.write(f"Raw 10 bytes hex: {bytes(result['raw_bytes']).hex(' ')}\n")
        f.write(f"Header: 0x{result['header']:02X}\n")
        f.write(f"Packet ID: {result['packet_id']}\n")
        f.write(f"Last packet: {result['last_packet']}\n")
        f.write(f"Header len field: {result['len_field']}  (0 means 8 bytes)\n")
        f.write(f"Payload fixed length: {result['payload_len']}\n")
        f.write(f"Payload 8 bytes hex: {bytes(result['payload']).hex(' ')}\n")
        f.write(f"Useful payload hex after stripping 0x00: {result['useful_payload'].hex(' ')}\n")
        f.write(f"Received CRC: 0x{result['received_crc']:02X}\n")
        f.write(f"Calculated CRC: 0x{result['calculated_crc']:02X}\n")
        f.write(f"CRC OK: {result['crc_ok']}\n")
        f.write(f"Decoded text: {result['text']}\n")

    print(f"[SAVE] Decode result saved to: {path}")


def save_hamming_decode_report(result, filename):
    path = os.path.join(RUN_DIR, filename)

    with open(path, "w", encoding="utf-8") as f:
        f.write("========== Detailed Hamming Decode Report ==========\n")
        f.write(
            "index,byte_index,nibble_position,"
            "original_codeword,syndrome,corrected,error_bit_name,"
            "corrected_codeword,decoded_nibble_bits,nibble_hex,byte_hex\n"
        )

        for item in result["decode_report"]:
            byte_value = item["byte_value"]

            if byte_value is None:
                byte_hex = ""
            else:
                byte_hex = f"0x{byte_value:02X}"

            f.write(
                f"{item['index']},"
                f"{item['byte_index']},"
                f"{item['nibble_position']},"
                f"{item['original_codeword']},"
                f"{item['syndrome_str']},"
                f"{item['corrected']},"
                f"{item['error_bit_name']},"
                f"{item['corrected_codeword']},"
                f"{item['decoded_nibble_bits']},"
                f"0x{item['nibble_value']:X},"
                f"{byte_hex}\n"
            )

    print(f"[SAVE] Hamming decode report saved to: {path}")


def save_full_message():
    path = os.path.join(RUN_DIR, "full_message_result.txt")

    final_bytes = bytes(full_payload_bytes).rstrip(b"\x00")

    try:
        final_text = final_bytes.decode("ascii", errors="replace")
    except Exception:
        final_text = ""

    with open(path, "w", encoding="utf-8") as f:
        f.write("========== Full Message Result ==========\n")
        f.write(f"Packet order: {received_packet_order}\n")
        f.write(f"Full bytes hex: {final_bytes.hex(' ')}\n")
        f.write(f"Full decoded text: {final_text}\n")

    print("\n========== Full Message Result ==========")
    print(f"Packet order: {received_packet_order}")
    print(f"Full bytes hex: {final_bytes.hex(' ')}")
    print(f"Full decoded text: {final_text}")
    print("=========================================\n")
    print(f"[SAVE] Full message saved to: {path}")



async def ask_prompt_and_wait_pc_ready():
    prompt = input("Enter prompt for PC/Codex: ").strip()

    while not prompt:
        prompt = input("Prompt cannot be empty. Enter prompt: ").strip()

    print(f"[PC-BT] Connecting to PC {PC_BLUETOOTH_ADDRESS} on RFCOMM channel {PC_RFCOMM_CHANNEL}...")
    print("[PC-BT] PC will answer, flash ESP32, then send RECEIVED before decode starts.")

    def send_prompt_over_bluetooth():
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        sock.settimeout(PC_BLUETOOTH_TIMEOUT_SEC)

        try:
            sock.connect((PC_BLUETOOTH_ADDRESS, PC_RFCOMM_CHANNEL))
            sock.sendall((prompt + "\n").encode("utf-8"))

            chunks = []
            while True:
                data = sock.recv(1024)
                if not data:
                    break

                chunks.append(data)
                if b"\n" in data:
                    break

            return b"".join(chunks).decode("utf-8", errors="replace").strip()
        finally:
            sock.close()

    pc_reply = await asyncio.to_thread(send_prompt_over_bluetooth)

    if not pc_reply.startswith("RECEIVED:"):
        raise RuntimeError(f"PC did not confirm readiness: {pc_reply}")

    pc_response = pc_reply.split(":", 1)[1]
    print(f"[PC-BT] PC confirmed ESP32 is flashed. Response = {pc_response}")

    prompt_path = os.path.join(RUN_DIR, "prompt.txt")
    with open(prompt_path, "w", encoding="utf-8") as f:
        f.write(prompt + "\n")

    print(f"[SAVE] Prompt saved to: {prompt_path}")
    return prompt

async def send_prompt_and_wait_ready(ble_client, prompt):
    prompt_ack = asyncio.Event()

    def handle_tx_notify(_, data):
        msg = data.decode("utf-8", errors="replace").strip()
        if msg:
            print(f"[BLE] ESP32 notify: {msg}")

        if msg.startswith("PROMPT_ACK:"):
            prompt_ack.set()

    await ble_client.start_notify(TX_CHAR_UUID, handle_tx_notify)

    prompt_msg = f"PROMPT:{prompt}\n"
    await ble_client.write_gatt_char(
        RX_CHAR_UUID,
        prompt_msg.encode("utf-8"),
        response=False
    )

    print(f"[BLE] Sent prompt release to ESP32-S3: {prompt}")
    print("[BLE] Waiting for PROMPT_ACK before starting decode...")

    try:
        await asyncio.wait_for(prompt_ack.wait(), timeout=PROMPT_ACK_TIMEOUT_SEC)
    except asyncio.TimeoutError:
        raise TimeoutError("Timed out waiting for PROMPT_ACK:RECEIVED from ESP32-S3")

    print("[BLE] Prompt ACK received. Start realtime decode.")

# ============================================================
# BLE ACK helper
# ============================================================
async def send_ack_ble(ble_client, packet_id):
    ack_msg = f"ACK:{packet_id}\n"

    await ble_client.write_gatt_char(
        RX_CHAR_UUID,
        ack_msg.encode(),
        response=False
    )

    print(f"[BLE] Sent ACK to ESP32-S3: {ack_msg.strip()}")


# ============================================================
# Realtime RX state machine
# ============================================================
def reset_realtime_search():
    global rx_state
    global encoded_buffer
    global preamble_buffer

    rx_state = STATE_SEARCH_PREAMBLE
    encoded_buffer = []
    preamble_buffer.clear()


async def process_realtime_bit(bit, ble_client):
    global rx_state
    global encoded_buffer
    global detected_preamble_count
    global crc_fail_count
    global decoded_done
    global last_packet_received

    if rx_state == STATE_SEARCH_PREAMBLE:
        preamble_buffer.append(bit)

        matched, dist = check_preamble(preamble_buffer)

        if matched:
            detected_preamble_count += 1

            print("\n[SYNC] Preamble detected!")
            print(f"[SYNC] Preamble = {''.join(preamble_buffer)}")
            print(f"[SYNC] Preamble error = {dist}")
            print("[RX] Start receiving encoded 140 bits...")

            encoded_buffer = []
            rx_state = STATE_RECEIVE_ENCODED

    elif rx_state == STATE_RECEIVE_ENCODED:
        encoded_buffer.append(bit)

        if len(encoded_buffer) >= ENCODED_BITS:
            encoded_bits = "".join(encoded_buffer)

            print("\n[RX] Encoded bits received.")
            print("[RX] Start Hamming decode + CRC check...")

            try:
                result = decode_encoded_bits(encoded_bits)

                if result is None:
                    print("[ERROR] Decode returned None. Back to SEARCH_PREAMBLE.")
                    reset_realtime_search()
                    return None

                if result["crc_ok"]:
                    packet_id = result["packet_id"]

                    print("[CRC] CRC OK.")

                    print_decode_result(result)
                    print_hamming_decode_report(result)

                    save_decode_result(
                        result,
                        filename=f"packet_{packet_id:02d}_decoded_result.txt"
                    )

                    save_hamming_decode_report(
                        result,
                        filename=f"packet_{packet_id:02d}_hamming_report.txt"
                    )

                    if packet_id not in received_packets:
                        received_packets[packet_id] = result
                        received_packet_order.append(packet_id)
                        full_payload_bytes.extend(result["useful_payload"])
                    else:
                        print(
                            f"[WARN] Duplicate packet_id={packet_id}. "
                            "ACK again, but do not append payload."
                        )

                    await send_ack_ble(ble_client, packet_id)

                    if result["last_packet"]:
                        print("[DONE] Last packet received.")
                        last_packet_received = True
                        decoded_done = True
                        save_full_message()
                        return result

                    print("[RX] Waiting for next packet...")
                    reset_realtime_search()
                    return result

                crc_fail_count += 1

                print("[CRC] CRC FAIL. No ACK will be sent.")
                print("[RX] Back to SEARCH_PREAMBLE.")
                print(f"[CRC] Received CRC: 0x{result['received_crc']:02X}")
                print(f"[CRC] Calculated CRC: 0x{result['calculated_crc']:02X}")
                print(f"[DEBUG] Raw 10 bytes hex: {bytes(result['raw_bytes']).hex(' ')}")

                print_hamming_decode_report(result)

                fail_filename = f"realtime_hamming_crc_fail_{crc_fail_count:03d}.txt"
                save_hamming_decode_report(result, filename=fail_filename)

                reset_realtime_search()
                return None

            except Exception as e:
                crc_fail_count += 1
                print(f"[ERROR] Decode failed: {e}")
                print("[RX] Back to SEARCH_PREAMBLE.")
                reset_realtime_search()
                return None

    return None


# ============================================================
# Main async
# ============================================================
async def main_async():
    global frame_count
    global raw_bit_log
    global decoded_done

    print(f"[SAVE] All results will be saved under: {RUN_DIR}")

    prompt = await ask_prompt_and_wait_pc_ready()

    print(f"[BLE] Connecting to ESP32-S3: {ESP32_ADDRESS}")

    async with BleakClient(ESP32_ADDRESS) as ble_client:
        print("[BLE] Connected to ESP32-S3.")
        print("[BLE] Prompt/status notify channel ready.")
        print("[BLE] ACK write channel ready.")

        await send_prompt_and_wait_ready(ble_client, prompt)

        reset_camera()
        Gst.init(None)

        pipeline = None

        try:
            pipeline_desc = create_pipeline()
            print("GStreamer pipeline:")
            print(pipeline_desc)

            pipeline = Gst.parse_launch(pipeline_desc)
            sink = pipeline.get_by_name("sink")

            if sink is None:
                print("Could not find appsink 'sink' in pipeline.")
                return

            ret = pipeline.set_state(Gst.State.PLAYING)

            if ret == Gst.StateChangeReturn.FAILURE:
                print("Failed to start pipeline")
                return

            print("\nCamera started.")
            print(f"Resolution: {CAP_WIDTH} x {CAP_HEIGHT}")
            print(f"FPS setting: {FPS}")
            print(f"Shutter speed: 1/600 s")
            print(f"Exposure time: {EXPOSURE_NS} ns")
            print(f"Analog gain: {ANALOG_GAIN}")
            print(f"ISP digital gain: {ISP_DIGITAL_GAIN}")
            print()
            print("========== Realtime RX Protocol ==========")
            print(f"Preamble: {PREAMBLE_BITS}")
            print(f"Preamble bits: {PREAMBLE_LEN}")
            print(f"Header bytes: {HEADER_BYTES}")
            print(f"Payload bytes: {PAYLOAD_BYTES}")
            print(f"CRC bytes: {CRC_BYTES}")
            print(f"Raw bytes: {RAW_BYTES}")
            print(f"Raw bits: {RAW_BITS}")
            print(f"Nibble count: {NIBBLE_COUNT}")
            print(f"Encoded bits: {ENCODED_BITS}")
            print(f"Total TX bits: {TX_BITS_TOTAL}")
            print(f"MAX_PREAMBLE_ERROR: {MAX_PREAMBLE_ERROR}")
            print("==========================================")
            print()
            print("Realtime receiving started after PROMPT_ACK.")
            print("CRC OK -> send ACK:<packet_id> by BLE.")
            print("CRC FAIL -> no ACK, wait for retransmission.")
            print("Last packet -> stop and save full message.")
            print("Press q or ESC to quit.\n")

            start_time = time.perf_counter()

            while True:
                sample = sink.emit("pull-sample")

                if sample is None:
                    print("No sample received from appsink")
                    continue

                buf = sample.get_buffer()

                caps = sample.get_caps()
                structure = caps.get_structure(0)

                width = structure.get_value("width")
                height = structure.get_value("height")

                success, map_info = buf.map(Gst.MapFlags.READ)

                if not success:
                    print("Failed to map buffer")
                    continue

                try:
                    frame = np.frombuffer(
                        map_info.data,
                        np.uint8
                    ).reshape((height, width, 3)).copy()
                finally:
                    buf.unmap(map_info)

                frame_count += 1

                bit, mean_gray, bright_ratio, roi_box = analyze_led_state(frame)

                raw_bit_log += bit

                await process_realtime_bit(bit, ble_client)

                state_text = rx_state

                preview_frame = draw_roi_overlay(
                    frame,
                    bit,
                    mean_gray,
                    bright_ratio,
                    roi_box,
                    state_text
                )

                elapsed = time.perf_counter() - start_time
                actual_fps = frame_count / elapsed if elapsed > 0 else 0

                cv2.putText(
                    preview_frame,
                    f"Realtime RX + BLE ACK | frame={frame_count} | FPS={actual_fps:.2f}",
                    (30, height - 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )

                cv2.putText(
                    preview_frame,
                    "CRC OK -> ACK | Last packet -> Stop | Press q to quit",
                    (30, height - 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    2,
                )

                cv2.putText(
                    preview_frame,
                    f"preamble={PREAMBLE_LEN} | encoded={ENCODED_BITS} | total={TX_BITS_TOTAL}",
                    (30, height - 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    2,
                )

                cv2.imshow("Realtime CSI RX with BLE ACK", preview_frame)

                key = cv2.waitKey(1) & 0xFF

                if decoded_done:
                    print("[DONE] All packets received. Stop realtime RX.")
                    break

                if key == 27 or key == ord("q"):
                    print("[QUIT] User quit.")
                    break

        except KeyboardInterrupt:
            print("Interrupted by user.")

        finally:
            save_raw_bit_log()

            cv2.destroyAllWindows()

            if pipeline is not None:
                pipeline.set_state(Gst.State.READY)
                time.sleep(0.2)
                pipeline.set_state(Gst.State.NULL)
                time.sleep(0.5)

            print(f"[SAVE] All output files are in: {RUN_DIR}")
            print("Pipeline stopped, exiting.")


if __name__ == "__main__":
    asyncio.run(main_async())
