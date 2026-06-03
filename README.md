## How It Works

This project is an ESP32-S3 firmware bridge from Autosport Labs (https://wiki.autosportlabs.com/ESP32-CAN-X2) that listens to ASL's TireX CAN1 bus and forwards selected frames over Bluetooth LE to RaceChrono's DIY CAN-over-BLE protocol (https://racechrono.com/article/2572).

Rough feature set:

1. The ESP32 receives CAN frames on CAN1.
2. CAN frames are treated with a filter. Either all CAN frames are forwarded, or just the default Tirex PID's are forwarded.
3. Frames passing the filter are sent to RaceChrono via Bluetooth LE notification.

Build Flags:
The firmware has some features you can enable or disable:
- Enable live stream of Tirex frame data via the ESP32 console.
- **Forward all CAN PIDs** for raw passthrough and diagnostics. Or you just got a lot of CANBUS to send!
- **RaceChrono filter mode** for forwarding only selected PIDs, while still allowing TireX frames through by default.

If needed, configure RaceChrono CAN filters to request specific PIDs.
Watch the serial log for connection status and forwarded frames.

## How to Use

1. Choose the build flags with `idf.py menuconfig` command before building. Menuconfig is part of the esp-idf build tools (https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#idf-py):
   - Enable `TIREX_FORWARD_ALL_PIDS_OVER_BLE` to forward every CAN frame over Bluetooth. Maybe you have more devices than TireX you want to send
   - Leave it disabled for normal filtered behavior. In that mode, TireX CAN IDs are still forwarded by default, and additional PIDs can be enabled through RaceChrono’s filter characteristic. Good if you have limited ble bandwidth.
   - Use `TIREX_DISABLE_ALL_CONSOLE_LOGGING` if you want a quiet serial monitor.
   - Use `TIREX_DECODER_CONSOLE_LOGGING` if you want TireX temperature values printed to the console while CAN frames are received.
2. Flash the firmware to the ESP32-S3. 'idf.py builf flash monitor'
2. Power the board and confirm from your Bluetooth pairing mode that the board is visible as `RC_TireX`. The process is similar to paring a headset or speakers to your phone.
3. Open RaceChrono and add a DIY device to the vehicle profile. You should see your RC_Tirex device under 'Bluetooth LE' connections.
4. Configure your canbus PID' in the racechrono vehicle settings.


## If You Just Want to Try This on Your ESP32 and are about that tldr/yolo life

If you already have the compiled binary files (check this repo's releases section), you can flash them directly without setting up the full ESP-IDF build environment.

Use `esptool.py` to write the bootloader, partition table, and app image to the board:

```bash
python3 -m esptool --chip esp32s3 -b 460800 \
  --before default-reset --after hard-reset \
  write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/tirex_racechrono_ble.bin
```
## How NOT to use.
1. don't try to power your tirex sensors from the ESP32 board. I released the magic smoke on one of my boards doing this
2. don't feed your board 12v from anything other than the 20/19 pins (or as documented).
