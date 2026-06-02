## How It Works

This project is an ESP32-S3 firmware bridge that listens to TireX CAN traffic on the vehicle CAN bus and forwards selected frames to RaceChrono over Bluetooth LE using RaceChrono’s DIY CAN-over-BLE protocol.

At a high level:

1. The ESP32 receives extended CAN frames on CAN1/TWAI.
2. TireX frames can be decoded locally for debug logging.
3. A filter decides which CAN IDs should be forwarded over BLE.
4. Matching frames are sent to RaceChrono as BLE notifications.

The firmware supports two forwarding modes:

- **Forward all CAN PIDs** for raw passthrough and diagnostics.
- **RaceChrono filter mode** for forwarding only selected PIDs, while still allowing TireX frames through by default.

The BLE service advertises as `RC_TireX` and exposes the standard RaceChrono DIY CAN service and filter characteristic.

## How to Use

1. Flash the firmware to the ESP32-S3.
2. Power the board and confirm it is advertising as `RC_TireX`.
3. Open RaceChrono on Android and connect to the device.
4. Choose the forwarding mode in `menuconfig` before building:
   - Enable `TIREX_FORWARD_ALL_PIDS_OVER_BLE` to forward every extended CAN frame over Bluetooth. This is best for diagnostics and broad sensor capture.
   - Leave it disabled for normal filtered behavior. In that mode, TireX CAN IDs are still forwarded by default, and additional PIDs can be enabled through RaceChrono’s filter characteristic.
   - Use `TIREX_DISABLE_ALL_CONSOLE_LOGGING` if you want a quiet serial monitor.
   - Use `TIREX_DECODER_CONSOLE_LOGGING` if you want TireX temperature values printed to the console while CAN frames are received.
5. If needed, configure RaceChrono CAN filters to request specific PIDs.
6. Watch the serial log for connection status and forwarded frames.

## If You Just Want to Try This on Your ESP32

If you already have the compiled binary files, you can flash them directly without setting up the full ESP-IDF build environment.

Use `esptool.py` to write the bootloader, partition table, and app image to the board:

```bash
python3 -m esptool --chip esp32s3 -b 460800 \
  --before default-reset --after hard-reset \
  write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/tirex_racechrono_ble.bin
