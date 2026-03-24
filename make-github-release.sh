#!/bin/bash -e

if [[ "$#" -eq 1 ]];  then
  version="$1"
else
  version=$(date --iso-8601)
fi

name=$(echo clock4-$version.zip)

fwtver=$(tail -c 64 qspi/output/fwt.bin | head -c 40)
fwdver=$(tail -c 64 qspi/output/fwd.bin | head -c 40)
bootver=$(tail -c 64 mk4-bootloader/Release/mk4-bootloader.bin | head -c 40)

echo "Releasing $name"
echo
echo "- \`tzmap\` and \`tzrules\` are ..."
echo "- \`fwt\` is \`$fwtver\`"
echo "- \`fwd\` is \`$fwdver\`"
echo "- \`bootloader\` is \`$bootver\`"
echo


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

echo "Done"
