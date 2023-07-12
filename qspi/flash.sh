#!/bin/bash -e

dev=$(ls /dev/disk/by-id/usb-mitxela_Precision_Clock* | head -1)
[[ -z "$dev" ]] && ( echo "No device found"; exit 1 )

dev=$(readlink -f "$dev") 
echo "Device $dev"


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
sudo mount "$dev" mnt/

# each file individually is optional, warn but continue
sudo cp config.txt mnt/ || true
sudo cp output/tzrules.bin mnt/ || true
sudo cp output/tzmap.bin mnt/ || true
sudo cp output/fwt.bin mnt/ || true
sudo cp output/fwd.bin mnt/ || true

sync_progress
ls -alh mnt/

