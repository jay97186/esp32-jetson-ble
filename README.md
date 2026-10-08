# ESP32–Jetson Optical Communication with BLE ACK

The ESP32-S3 transmits data by turning a NeoPixel LED on and off, while the Jetson receives the optical signals in real time through a CSI camera. The Jetson performs Hamming(7,4) decoding and CRC-8 verification, then sends packet acknowledgments (ACKs) back via BLE.

This repository contains the source code and experimental data from May 21, 2026.

## File Structure

| Path | Description |
| --- | --- |
| `firmware/mypcesp32tx/mypcesp32tx.ino` | ESP32-S3 LED transmitter, BLE status notifications, and ACK reception |
| `receiver/jetsonrx.py` | Jetson CSI camera receiver; originally named `jetsonrx (2).py` |
| `experiments/realtime_ble_ack_run_20260521_225857/` | 15 original experimental records with filenames preserved |
| `media/demo.mp4` | Original demonstration video, renamed for easier access |
| `SOURCE_MANIFEST.csv` | Original filenames, Google Drive sources, organized paths, file sizes, and SHA-256 checksums |

## System Workflow

1. The Jetson reads user input and sends it to the PC via Bluetooth RFCOMM.
2. The PC responds with `RECEIVED:` to confirm that the ESP32 firmware is ready.
3. The Jetson sends `PROMPT:` via BLE. After the ESP32 responds with `PROMPT_ACK:RECEIVED`, LED transmission begins.
4. The Jetson decodes each packet. If the CRC check passes, it sends `ACK:<packet_id>`; otherwise, it waits for retransmission.
5. After receiving the final packet, the Jetson saves the complete message, decoding reports, and raw bit logs.

**Note:** The PC-side RFCOMM service and the firmware generation/flashing programs are not included in the source folder. Therefore, the current repository is insufficient to independently reproduce the complete PC → ESP32 → Jetson workflow.

## Hardware and Dependencies

- **ESP32-S3 with NeoPixel LED:** Original configuration uses GPIO 48, one LED, and brightness level 30.
- **ESP32 software:** Arduino ESP32 development environment, ESP32 BLE headers, and Adafruit NeoPixel library.
- **Jetson software:** Jetson Linux, CSI camera, Bluetooth, NVIDIA Argus/GStreamer, PyGObject, NumPy, OpenCV, and Bleak.
- **PC-side service:** Bluetooth RFCOMM service on channel 4, configured to respond with `RECEIVED:`.

The original source does not specify library versions, JetPack version, or exact development board and camera models.

## Usage Instructions

1. Open `firmware/mypcesp32tx/mypcesp32tx.ino` in the Arduino IDE. Verify the development board and LED GPIO settings, install the required libraries, and flash the firmware.
2. On the Jetson, modify `ESP32_ADDRESS`, `PC_BLUETOOTH_ADDRESS`, `PC_RFCOMM_CHANNEL`, `SAVE_DIR`, and the camera/ROI settings in `receiver/jetsonrx.py`.
3. Set up the PC-side RFCOMM service and verify the Bluetooth connection.
4. Run the receiver on the Jetson:

   ```bash
   python3 receiver/jetsonrx.py
   ```

5. Enter the prompt. By default, results are saved to:

   `/home/jetson/capture_output/realtime_ble_ack_run_<timestamp>/`

   Press `q` or `Esc` in the preview window to exit.

**Important notes:**

- The receiver executes `sudo systemctl restart nvargus-daemon`. The execution environment must permit this operation.
- The original Bluetooth addresses correspond to experimental devices and must be updated before deployment.

## Communication Protocol

| Parameter | Original Configuration |
| --- | --- |
| Preamble | `1111100110101` (13 bits) |
| Header | 1 byte: 4-bit packet ID, 1-bit final-packet flag, 3-bit length field |
| Payload | 8 bytes per packet; length field 0 represents 8 bytes |
| CRC | CRC-8, polynomial `0x07`, initialization `0x00` |
| Error Correction | Hamming(7,4), 20 codewords, resulting in 140 interleaved bits |
| Total Packet Length | 153 bits, including preamble |
| LED Bit Interval | 16,667 µs (approximately 60 bits/s) |
| Camera Settings | 1280 × 720, 60 FPS, approximately 1/600-second exposure |
| ACK Timeout | 5 seconds |

## Experimental Results

The saved `full_message_result.txt` records the following packet sequence:

`[0, 1, 2, 3, 4]`

The reconstructed message is:

`I don't have an age; I'm Codex, an AI.`

The experiment folder also contains:

- Individual packet decoding reports and Hamming decoding reports
- Two CRC failure reports
- Raw bit logs
- The original prompt

These are historical experimental results preserved from the source files. No hardware transmission tests were performed during this repository organization process.

The source files were preserved byte-for-byte. Some firmware comments already contained corrupted characters in the original files.

## Source

[Original Google Drive Folder](https://drive.google.com/drive/folders/120jVGPg3Axbpoq-2wb5N0mOmq_TNrsM0)

No additional license terms have been specified.
