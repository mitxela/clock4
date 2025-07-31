# QSPI flash contents
The external flash memory is exposed as a USB MSC device directly. It's 16MB and contains the config, timezone rules, timezone shapefile (map), and optionally firmware images for both microcontrollers.

## Format
The memory must be formatted FAT12 or FAT16, with block size of 4096 (not 512).

To reformat and erase the device (requires `dosfstools` installed):
```
sudo mkfs.fat -I -S 4096 /dev/sdx
```
where `/dev/sdx` is the device name

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
