## What is this project?

I always wanted a way to capture vehicle data from a Porsche without being locked into the Porsche Track Precision app.

Through the wifi hotspot and a 12v vehicle outlet, this projects lets me capture signals like:

| Porsche Data|
| ----------- |
| lat/long Acceleration      | 
| Understeer/Oversteer   |
|Brake Pressure|
|Wheel Angle|
|Yaw Rate|
|Vehicle Speed|
|Tyre Velocities|
|Accelerator Position|
| Engine Speed|
| Current Gear|

This project also can read CANBUS off the wire and transport it wirelessly over Bluetooth.

**Current Features:**
Wireless capture of Porsche EXLAP data (same data source from Porsche Precision Track app)
EXLAP & CANBUS -> Bluetooth LE -> Racechrono Pro (via DIY Bluetooth feature)
Autosportlabs Tirex sensor configuration through webUI

**Planned Work(w)/Features(f):**
(f)Improved Tirex sensor installation walkthrough
(w)Better documentation
(w)Stress testing to determine CANBUS->BLE throughput limits and alerting when it breaks
(w)Measure latency between between sensors creating data and when its recorded in the data logger. ie. How accurate is this data? 

## How it works
I use an ESP32-S3 to process CANBUS data and intrepert EXLAP from the cars wifi hotspot. The hardware is provided by Autosport Labs (https://wiki.autosportlabs.com/ESP32-CAN-X2). Once processed, we send selected data over Bluetooth LE to RaceChrono's DIY CAN-over-BLE protocol (https://racechrono.com/article/2572). This puts all our telemetry onto RaceChrono and from there we can analys or export it into various formats like CircuitTools, CSV

Build Flags:
A lot of the build flags are going to change as I build up new features. They a

If needed, configure RaceChrono CAN filters to request specific PIDs.

Watch the serial log for connection status and forwarded frames.

## How to Use

1. Choose the build flags with `idf.py menuconfig` command before building. Menuconfig is part of the esp-idf build tools (https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#idf-py):
   - Enable `TIREX_FORWARD_ALL_PIDS_OVER_BLE` to forward every CAN frame over Bluetooth. Maybe you have more devices than TireX you want to send
   - Leave it disabled for normal filtered behavior. In that mode, TireX CAN IDs are still forwarded by default, and additional PIDs can be enabled through RaceChrono’s filter characteristic. Good if you have limited ble bandwidth.
   - Use `TIREX_DISABLE_ALL_CONSOLE_LOGGING` if you want a quiet serial monitor.
   - Use `TIREX_DECODER_CONSOLE_LOGGING` if you want TireX temperature values printed to the console while CAN frames are received.
2. Flash the firmware to the ESP32-S3. 'idf.py build flash monitor'
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
