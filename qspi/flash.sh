#!/bin/bash -e

if [[ "$1" == "loop" ]]; then
  echo "Creating disk image"
  dev="temp.img"
  rm -f "$dev"

  # fill with 0xFF
  tr '\0' '\377' < /dev/zero | dd bs=4K of="$dev" count=4096
else
  dev=$(ls /dev/disk/by-id/usb-mitxela_Precision_Clock* | head -1)
  [[ -z "$dev" ]] && ( echo "No device found"; exit 1 )

  dev=$(readlink -f "$dev") 
  echo "Device $dev"
fi
 
function sync_progress {
  sync &
  proc=$!

  while kill -0 "$proc" >/dev/null 2>&1; do
    d=$(grep Dirty: /proc/meminfo)
    w=$(grep Writeback: /proc/meminfo)
    echo -ne "\r"$d "\t" $w "      "
    sleep 0.5
  done
  echo
}


mkdir -p mnt
function cleanup {
  sudo umount mnt/ || true
  rm -rf mnt/
}
trap cleanup EXIT

sudo mkfs.fat -I -S 4096 "$dev"
sudo fatlabel "$dev" CLOCK
sudo mount "$dev" mnt/ # auto detects if loop device


# Windows creates a folder with restore point info, just wastes space and causes
# spurious writes. Suppress it by making a hidden file with the same name
sudo touch 'mnt/System Volume Information'

# mark it hidden
sudo fatattr +h 'mnt/System Volume Information' || echo "fatattr failed"

# SETTINGS.BIN: the firmware's on-device settings store (menu overrides + learned tempcomp model) on
# 256K-silicon clocks, which have no spare internal flash. It MUST be created FIRST on the fresh
# volume so its 4 clusters are contiguous (the firmware verifies contiguity and falls back to
# RAM-only settings if the file is fragmented), and filled with 0xFF (NOR erased state). The firmware
# rewrites the file's sectors in place; treat it as opaque and don't copy/edit it from the host.
tr '\0' '\377' < /dev/zero | dd bs=4096 count=4 2>/dev/null | sudo tee mnt/SETTINGS.BIN > /dev/null

# each file individually is optional, warn but continue
sudo cp config.txt mnt/ || true
sudo cp output/tzrules.bin mnt/ || true
sudo cp output/tzmap.bin mnt/ || true
sudo cp output/fwt.bin mnt/ || true
sudo cp output/fwd.bin mnt/ || true


if [[ "$1" == "loop" ]]; then
  mv "$dev" output/clock-qspi.img
  echo "Created output/clock-qspi.img"
else
  sync_progress
fi
ls -alh mnt/
