# QSPI flash contents
The external flash memory is exposed as a USB MSC device directly. It's 16MB and contains the config, timezone rules, timezone shapefile (map), and optionally firmware images for both microcontrollers.

## Format
The memory must be formatted FAT12 or FAT16, with block size of 4096 (not 512).

To reformat and erase the device (requires `dosfstools` installed):
```
sudo mkfs.fat -I -S 4096 /dev/sdx
```
where `/dev/sdx` is the device name

## SETTINGS.BIN
On clocks built with 256K-flash silicon (STM32L476RC) the firmware has no spare internal flash for
its settings store (on-device menu changes + the learned temperature-compensation model), so it
keeps them inside `SETTINGS.BIN` on this volume instead: a 16 KiB file whose four 4 KB clusters the
firmware rewrites *in place* through the QSPI driver, without ever writing FAT metadata. Rules:

- Create it **first** on a freshly formatted volume (flash.sh does this) so it is contiguous -- the
  firmware verifies contiguity at boot and falls back to RAM-only settings (lost at power-off) if
  the file is fragmented, missing, or under 16 KiB.
- Fill it with `0xFF` (the NOR erased state): `tr '\0' '\377' < /dev/zero | dd bs=4096 count=4 of=SETTINGS.BIN`.
  A zero-filled file also works -- the firmware erases it on first use -- but 0xFF is the honest state.
- Treat it as **opaque**: don't edit, copy over, or defragment it. If a host tool rewrites or moves
  it, the firmware detects this before its next write, re-resolves the file's location, and
  re-initialises it (stored settings reset to the live values; nothing else on the volume is touched).
- `menu_dump = on` over serial reports the store state: internal-flash-backed (1M silicon),
  QSPI SETTINGS.BIN, or RAM-only with the reason.

1M-silicon (STM32L476RG) clocks ignore the file entirely and keep using the internal-flash store.

## tzmap
The `tzmap.bin` file matches the format used by [ZoneDetect](https://github.com/BertoldVdb/ZoneDetect). The repo has a directory with the database builder.

To run the builder install shapelib (and I had to add `#include <cstdint>`)

Rename `out/timezone21.bin` to `tzmap.bin`

The `out_v1/` format is smaller but takes longer to parse on the clock, so use the `out/` version.

Natural earth data is used only for the "country" database which isn't used here. All data comes from https://github.com/evansiroky/timezone-boundary-builder

## tzrules
The `tzrules.bin` file has a custom binary format parsed by the clock. For each timezone it lists the offsets and transition times up to the year 2100.

We need to match the zones in the shapefile, so it now uses the `timezone-names.json` file from the timezone-boundary-builder release. However the shapefile updates less frequently than the tzdb, so the versions may not match.

E.g.
```
curl -L https://github.com/evansiroky/timezone-boundary-builder/releases/download/2025b/timezone-names.json | jq '.' > timezone-names.json
```

The `generate-tzrules.py` file uses these names and the installed timezone database on the system it's running. Query the tzdata package to see the version (`pacman -Q tzdata` or `apt show tzdata`)

## config
An example `config.txt` is provided. 

## firmware images
There are two separate firmware image files. They are copied to the internal flash memory of each microcontroller so can be safely deleted after the update (or left there, it doesn't matter).

`fwt.bin` is for the "time" side of the clock (STM32L476RG, 192K). This update is performed by the custom bootloader that resides in the first 64K.

`fwd.bin` is for the "date" side of the clock (STM32L010C6, 32K). This update is performed by the STM32L476, over the serial connection using the system bootloader on the STM32L010.

Both firmware images are unencrypted/unobfuscated, but the last 32bit word of each file is the CRC32 of the contents. For the specific polynomial and endianness see the `fw-crc.sh` script.

The update is performed if the CRC of the image in external flash is valid and different to the CRC of the loaded image, so downgrades are easy.

## loop device to create disk image
`./flash.sh loop` will create a correctly formatted 16MB disk image of the QSPI contents using a loop device. The resulting image can be written using `dd` or some other disk image utility, which may be more convenient on other platforms.
