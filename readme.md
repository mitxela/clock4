# Precision Clock Mk IV

Source code for the [Precision Clock Mk IV](https://mitxela.com/projects/precision_clock_mk_iv)

There are three software projects, compiled with STM32CubeIDE:

- mk4-time is the main clock source code running on the STM32L476
- mk4-date is the secondary display running on the STM32L010
- mk4-bootloader runs on the STM32L476 to do firmware updates

In the QSPI folder, there are scripts to create firmware images with CRC that the clock will recognise, along with the scripts to create the tzrules and create a valid disk image, see [these notes](qspi/qspi.md) for more info.

Timezone detection is ported from [ZoneDetect](https://github.com/BertoldVdb/ZoneDetect) by Bertold Van den Bergh and uses shapefile data from [Timezone Boundary Builder](https://github.com/evansiroky/timezone-boundary-builder) by Evan Siroky.
