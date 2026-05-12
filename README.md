# SC26re

> Reverse engineering tools and experimental firmware reimplementation for the Steam Controller (2026)

This is a work-in-progress firmware reimplementation project for the 2026 Steam Controller, internally referred to as Ibex/Triton.

It is not production firmware yet, and it is not an end-user mod package. But it is already good enough to play some games, despite missing gyro, haptics, and polished touchpad behavior. For example, I beat *Bowser in the Sky* with this firmware.

This project is unofficial and is not affiliated with, endorsed by, or supported by Valve Corporation.
This repository does not contain Valve firmware images.

## Status

Currently working at least partially:

- direct GPIO and GreenPAK-backed buttons
- analog sticks and triggers
- Olympus touchpad: basic support
- RGBW LED
- USB HID
- BLE HID (only with SoftDevice/MPSL)
- ESB (puck) HID
- ESB bonding over BLE/USB
- BLE/ESB/USB personality switching, including with boot chords
- power-button long-hold and sleep mode
- Ibex settings registry (mostly only storage)
- Valve bootloader-compatible packaging and flashing

The most glaring omissions are gyro, haptics, battery/charger handling, and grip sense.

### TODO

- [x] Olympus touchpad
  - [ ] Exactly OFW-matching pressure/click/deadzone behavior
- [x] Analog inputs
  - [ ] Trigger neutral deadzone seems too large
- [x] Buttons
  - [x] GreenPAK buttons
  - [ ] Grip sense
- [ ] Power
  - [x] Power-button long-hold system-off path
  - [x] Sleep mode
    - [ ] Verify that idle power consumption matches OFW
  - [ ] Battery level
  - [ ] Charger detection
- [ ] LEDs
  - [x] RGBW LED
  - [ ] IR LED
- [x] Ibex settings registry
  - [ ] Actually use the settings in the firmware
- [x] ESB
  - [x] ESB bonding over BLE
  - [x] ESB bonding over USB
  - [ ] Expose ESB bonds over HID commands
- [x] BLE
  - [ ] After startup/reconnect, Steam sometimes ignores input until the Steam button is pressed
- [x] Personality switching (BLE, ESB, USB)
  - [x] Over UART
  - [x] Over BLE, mainly for ESB bonding
  - [x] With R1+A / R1+B boot chords
    - [ ] Second ESB bond with L1+A
- [ ] Haptics
- [ ] Freefall detection
  - [ ] Wilhelm scream
- [ ] Puck / Proteus / Nereid
  - [ ] Puck detection/wired communication(?)
  - [ ] Puck firmware reimplementation

## Repository layout

- [`app/`](app/) - Zephyr application, out-of-tree Ibex board support, drivers, and the main firmware.
- `flash.py`, `fwtool.py` - host tools.
- `bootstub/` - experiment for jumping into the original Ibex_FW payload on a BBC micro:bit v2, which has the same SoC. This is not part of the normal custom firmware boot path.

## Bootloader entry chord

To keep the controller in Valve's resident bootloader instead of starting the application, hold:

```text
View + Menu + A
```

then press Steam to power on the controller.

Steam is just the power button in this sequence. The chord checked by the boot path is View + Menu + A.

## Building

The app README has more detailed build instructions. The minimal command is:

```sh
make app-ibex
```

There is also a BBC micro:bit v2 target, useful as a development stand-in:

```sh
make app-microbit
```

Read [`app/README.md`](app/README.md) before flashing anything. In particular, read the notes about the Valve bootloader layout, settings storage, and what survives a firmware update.

## Flashing

For a controller already in Valve ISP bootloader mode:

```sh
make app-ibex-bl-flash IBEX_BL_PORT=/dev/ttyACM0
```

For an initial flash from the USB HID runtime, the helper can request ISP mode and match the resulting bootloader, all by serial:

```sh
make app-ibex-flash IBEX_SERIAL=<controller serial>
```

Subsequent updates from the custom firmware can use the HID feature channel, or the `steamctl power reboot_isp` shell command, to return to the bootloader.

## Safety, warranty, etc.

Flashing this can temporarily or permanently break your controller if the assumptions are wrong, the build is wrong, your local changes are wrong, or something else is wrong. You should be comfortable with SWD, serial logs, and recovering an nRF52 board before using this on hardware you care about, especially since these controllers are so hard to get hold of.

This software comes with no warranty, express or implied, not even MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Use at your own risk. You are solely responsible for any damage to your hardware, software, or data.

> [!CAUTION]
> **Do not flash this if this is your only controller and you are not prepared to recover it with SWD!**

The custom firmware is laid out to avoid writing the stock firmware storage area (which has some annoying consequences for firmware updating), but that is not foolproof. If you modify it, do not carelessly use flash writing functions, and think twice before changing the flash layouts.

## Original firmware notes

The public NCS v2.9.0 release is used as the closest practical baseline for clean Zephyr builds. The firmware banners in the analyzed controller and puck images refer to Zephyr/NCS revisions that do not exist in the public upstream repositories.

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see https://www.gnu.org/licenses/.

Files under [app/src/sdl/](app/src/sdl/) are derived from SDL and retain SDL’s zlib-style license; see [app/src/sdl/LICENSE.txt](app/src/sdl/LICENSE.txt).

See [LICENSE](LICENSE) for details.
