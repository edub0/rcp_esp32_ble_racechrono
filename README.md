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

If you've already installed esptool from prior projects, you can build this board by downloading the latest .bin from the Release section of this page.

Put the board into boot mode - Power it on, hold down the 'Boot' button and then press 'Reset'. 
```
esptool --chip esp32s3 -b 460800 \
  --before default-reset \
  --after hard-reset \
  write-flash 0x0 merged-flash.bin
  ```
After flashing you may have to press the reset button to get the system up and working.

Wifi is 'Exlap-Gateway' // exlapgw1
Open a web browser, go to 192.168.4.1

You can see the status of the tirex connection, and enter your car's hotspot name and password, and exlap credentials.

## How NOT to use.
1. don't try to power your tirex sensors from the ESP32 board. I released the magic smoke on one of my boards doing this
2. don't feed your board 12v from anything other than the 20/19 pins (or as documented).
