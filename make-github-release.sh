#!/bin/bash -e

if [[ "$#" -eq 1 ]];  then
  version="$1"
else
  version=$(date --iso-8601)
fi

name=$(echo clock4-$version.zip)


function cleanup {
  rm -rf disk-image
  rm -rf bootloader
  rm -rf flash
}
trap cleanup EXIT


cleanup
mkdir disk-image
(
  cd qspi
  ./flash.sh loop
)
mv qspi/output/clock-qspi.img disk-image/clock4-$version.img

mkdir bootloader
cp mk4-bootloader/Release/mk4-bootloader.bin bootloader/mk4-bootloader-$version.bin

mkdir flash
cp qspi/output/fwt.bin flash/fwt.bin
cp qspi/output/fwd.bin flash/fwd.bin
cp qspi/output/tzrules.bin flash/tzrules.bin
cp qspi/output/tzmap.bin flash/tzmap.bin

zip -r $name disk-image bootloader flash

