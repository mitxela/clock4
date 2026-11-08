# Precision Clock Mk IV

Source code for the [Precision Clock Mk IV](https://mitxela.com/projects/precision_clock_mk_iv)

There are three software projects, compiled with STM32CubeIDE:

- mk4-time is the main clock source code running on the STM32L476
- mk4-date is the secondary display running on the STM32L010
- mk4-bootloader runs on the STM32L476 to do firmware updates

In the QSPI folder, there are scripts to create firmware images with CRC that the clock will recognise, along with the scripts to create the tzrules and create a valid disk image, see [these notes](qspi/qspi.md) for more info.

Timezone detection is ported from [ZoneDetect](https://github.com/BertoldVdb/ZoneDetect) by Bertold Van den Bergh and uses shapefile data from [Timezone Boundary Builder](https://github.com/evansiroky/timezone-boundary-builder) by Evan Siroky.

## 12-hour display mode

Set `MODE_12H=enabled` in the clock's `CONFIG.TXT` to select 12-hour local
time with the normal ISO date. The final (thousandths) digit shows `A` or `P`
for AM/PM, for example `01:23:45.67P`. Midnight is `12:00:00.00A` and noon is
`12:00:00.00P`. Tenths and hundredths retain the normal accuracy indication;
the AM/PM letter stays visible when fractional digits are hidden.

The buttons cycle through enabled modes. Leave `MODE_ISO8601_STD` enabled
for access to the original 24-hour display, or disable it to use only the
12-hour clock mode (plus any other enabled modes).

the "NEW-FIRMWARE" directory has the new fwt.bin in it for you, already compiled.
