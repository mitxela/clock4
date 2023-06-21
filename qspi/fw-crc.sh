#!/bin/bash -e

[[ "$#" -eq 2 ]] || { echo "Usage: $0 input.bin output.bin"; exit 1; }

file="$1"
out="$2"

[[ -f "$file" ]] || { echo "$file not found"; exit 1; }

[[ $( echo 41 | xxd -r -p ) == "A" ]] || { echo "xxd error"; exit 1; }

# using crc32 from package libarchive-zip-perl

# https://reveng.sourceforge.io/crc-catalogue/all.htm
# width=32 poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0xffffffff check=0xcbf43926 residue=0xdebb20e3 name="CRC-32/ISO-HDLC"

[[ "$(crc32 <(echo -n "123456789"))" == "cbf43926" ]] || { echo "CRC error"; exit 1; }

rm -f temp.bin
cp "$file" temp.bin

crc32 temp.bin | xxd -r -p >> temp.bin

rm -f "$out"
mv temp.bin "$out"

echo "Created $out"


# for alternate endianness can use
#  crc32 $file | fold -w2 | tac | xxd -r -p
