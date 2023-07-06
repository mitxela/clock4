#!/bin/bash

# Note for the debug build of mk4-time the linker script is different, would need to edit it to release a debug build
./fw-crc.sh ../mk4-time/Release/mk4-time.bin output/fwt.bin

# either config is fine for mk4-date
./fw-crc.sh ../mk4-date/Release/mk4-date.bin output/fwd.bin || \
./fw-crc.sh ../mk4-date/Debug/mk4-date.bin output/fwd.bin
