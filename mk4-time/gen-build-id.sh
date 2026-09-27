#!/bin/sh
# Pre-build hook (wired from the CubeIDE Release/Debug prebuildStep in .cproject, and from the
# generated Release/makefile pre-build rule). Stamps a fresh per-build id so every firmware image is
# byte-unique: the bootloader flashes on a CRC change, so an unchanged token would let two same-day
# 0.0.5 builds be byte-identical and get silently skipped. Regenerating build_id.h (new epoch) and
# removing version.o forces version.c to recompile with a fresh BUILD_ID + __DATE__/__TIME__ every build.
#
# Location-independent: cd to this script's own directory (the mk4-time project root) so it works no
# matter which build dir (Release/ or Debug/) invoked it.
cd "$(dirname "$0")" || exit 1
printf '#define BUILD_ID "%s"\n' "$(date +%s)" > Core/Src/build_id.h
rm -f Release/Core/Src/version.o Debug/Core/Src/version.o
