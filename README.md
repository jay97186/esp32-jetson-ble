# ESP32–Jetson 光通訊與 BLE ACK

ESP32-S3 以 NeoPixel LED 明滅傳送資料，Jetson 透過 CSI 相機即時接收，執行 Hamming(7,4) 解碼及 CRC-8 檢查，再透過 BLE 回傳封包 ACK。此儲存庫整理自 2026-05-21 的程式與實驗資料。

## 檔案導覽

| 路徑 | 內容 |
| --- | --- |
| `firmware/mypcesp32tx/mypcesp32tx.ino` | ESP32-S3 LED 傳送端、BLE 狀態通知與 ACK 接收 |
| `receiver/jetsonrx.py` | Jetson CSI 相機接收端；原始檔名為 `jetsonrx (2).py` |
| `experiments/realtime_ble_ack_run_20260521_225857/` | 15 份原始實驗紀錄，保留原檔名 |
| `media/demo.mp4` | 原始示範影片，重新命名便於查找 |
| `SOURCE_MANIFEST.csv` | 原始檔名、Drive 來源、整理後路徑、大小與 SHA-256 |

## 系統流程

1. Jetson 讀取使用者輸入，透過 Bluetooth RFCOMM 傳給 PC。
2. PC 回覆 `RECEIVED:`，確認 ESP32 韌體準備完成。
3. Jetson 透過 BLE 送出 `PROMPT:`；ESP32 回覆 `PROMPT_ACK:RECEIVED` 後開始 LED 傳輸。
4. Jetson 解碼封包；CRC 通過時回傳 `ACK:<packet_id>`，失敗時等待重傳。
5. 收到最後封包後，保存完整訊息、解碼報告與原始 bit log。

**PC 端 RFCOMM 服務及韌體產生／燒錄程式未包含於來源資料夾。** 因此現有檔案尚不足以獨立重現完整 PC → ESP32 → Jetson 流程。

## 硬體與相依項目

- ESP32-S3 與 NeoPixel LED。原始設定：GPIO 48、1 顆 LED、亮度 30。
- Arduino ESP32 開發環境、ESP32 BLE headers 與 Adafruit NeoPixel 函式庫。
- Jetson Linux、CSI 相機、Bluetooth、NVIDIA Argus／GStreamer、PyGObject、NumPy、OpenCV 與 Bleak。
- PC 端 Bluetooth RFCOMM 服務：channel 4，須回覆 `RECEIVED:`。

来源未記錄函式庫版本、JetPack 版本及確切開發板／相機型號。

## 使用方式

1. 以 Arduino IDE 開啟 `firmware/mypcesp32tx/mypcesp32tx.ino`，確認開發板與 LED GPIO，安裝相依函式庫後燒錄。
2. 在 Jetson 的 `receiver/jetsonrx.py` 調整 `ESP32_ADDRESS`、`PC_BLUETOOTH_ADDRESS`、`PC_RFCOMM_CHANNEL`、`SAVE_DIR` 與相機／ROI 設定。
3. 準備 PC 端 RFCOMM 服務並確認 Bluetooth 連線。
4. 在具有上述相依項目的 Jetson 環境執行：

   ```bash
   python3 receiver/jetsonrx.py
   ```

5. 輸入 prompt。預設結果存於 `/home/jetson/capture_output/realtime_ble_ack_run_<timestamp>/`。預覽視窗按 `q` 或 Esc 可結束。

接收程式會呼叫 `sudo systemctl restart nvargus-daemon`；執行環境須允許此操作。原始 Bluetooth 位址為實驗裝置設定，部署前須依實際設備修改。

## 傳輸協定

| 項目 | 原始設定 |
| --- | --- |
| Preamble | `1111100110101`，13 bits |
| Header | 1 byte：packet ID 4 bits、最後封包旗標 1 bit、長度欄位 3 bits |
| Payload | 每封包 8 bytes；長度欄位 0 表示 8 bytes |
| CRC | CRC-8，poly `0x07`、init `0x00` |
| 編碼 | Hamming(7,4)，20 個 codewords，交錯後 140 bits |
| 總封包長度 | 153 bits，含 preamble |
| LED bit interval | 16,667 µs，約 60 bits/s |
| 相機 | 1280 × 720、60 FPS、曝光約 1/600 秒 |
| ACK timeout | 5 秒 |

## 保存的實驗

`full_message_result.txt` 記錄封包順序 `[0, 1, 2, 3, 4]`，重組訊息為 `I don't have an age; I'm Codex, an AI.`。資料夾另包含每個封包的解碼與 Hamming 報告、2 份 CRC 失敗報告、原始 bit log 與 prompt。

這些是來源保存的歷史結果；本次整理沒有執行硬體傳輸測試。程式內容按原始 bytes 保存，部分韌體註解原本已有亂碼。

## 來源

[Google Drive 原始資料夾](https://drive.google.com/drive/folders/120jVGPg3Axbpoq-2wb5N0mOmq_TNrsM0)

未額外指定授權條款。
