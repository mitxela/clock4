#!/bin/bash -e

mkdir -p img-mnt

function cleanup {
  sudo umount img-mnt/
  rm -rf img-mnt/
}
trap cleanup EXIT

rm -f temp.img
#dd if=/dev/zero bs=16M of=temp.img count=1

# fill with 0xFF
tr '\0' '\377' < /dev/zero | dd bs=4K of=temp.img count=4096

mkfs.fat -I -S 4096 temp.img
sudo mount -o loop temp.img img-mnt/

# each file individually is optional, warn but continue
sudo cp config.txt img-mnt/ || true
sudo cp output/tzrules.bin img-mnt/ || true
sudo cp output/tzmap.bin img-mnt/ || true
sudo cp output/fwt.bin img-mnt/ || true
sudo cp output/fwd.bin img-mnt/ || true

mv temp.img output/clock-qspi.img
echo "Created output/clock-qspi.img"
