#!/usr/bin/env python3

import json, time, collections, subprocess, email.utils, calendar

# Use system tzdb
ZDUMP="/usr/bin/zdump"
# There's no obvious way to read which version of tzdata is installed
# except for pacman -Q tzdata or whatever

zoneNames = json.load(open('timezone-names.json'))
# e.g. https://github.com/evansiroky/timezone-boundary-builder/releases/download/2023b/timezone-names.json
# this includes the Etc/ zones

# wget https://github.com/evansiroky/timezone-boundary-builder/releases/download/2025b/timezones-with-oceans-now.geojson.zip
# unzip timezones-with-oceans-now.geojson.zip
# grep -o 'tzid":"[^"]*' combined-with-oceans-now.json | sort | uniq | cut -d\" -f3 | jq -Rn '[inputs]' > timezone-names-now.json
zoneNamesNow = json.load(open('timezone-names-now.json'))

# for paranoia
for zone in zoneNamesNow:
  if zone not in zoneNames:
    raise Exception("timezone-names-now.json has zone not in timezone-names.json")

zoneNames.sort() # input may not be grouped by prefix (e.g. Etc/UTC is on its own)

start = int(calendar.timegm(time.strptime("1 Jan 2020", "%d %b %Y")))
end = int(calendar.timegm(time.strptime("1 Jan 2100", "%d %b %Y")))

zones = collections.OrderedDict()
lookupAddr = {}
lookupCount = {}

output = open("output/tzrules.bin", "wb")
output.write(b'MTZ')       # header
output.write(b'\x01')      # version number
output.write(b'\x08')      # row length for fixed data

for zone in zoneNames:
  category, name = zone.split('/',1)

  if category not in zones:
    zones[category]=[]
  zones[category].append(name)

output.write(bytearray([len(zones)]))
endOfHeader = output.tell()

# Prototype data, fill in later
for category in zones:
  output.write(category.encode())
  output.write(b'\00xxy') # null, 16bit address, 8bit count

startOfNames = output.tell()
nameLut = {}

for category in zones:
  nameLut[category] = output.tell()
  for z in zones[category]:
    output.write(z.encode())
    output.write(b'\x00xxxy') # null, 24bit address, 8bit count

startOfData= output.tell()
output.seek(endOfHeader)

for category in zones:
  output.write(category.encode())
  output.write(bytearray([
    0,
    (nameLut[category] & 0x00FF ) >> 0,
    (nameLut[category] & 0xFF00 ) >> 8,
    len(zones[category])
    ]))

output.seek(startOfData)


# zdump -v output includes the second immediately before transition:
# Europe/London  Sun Mar 29 00:59:59 2020 UT = Sun Mar 29 00:59:59 2020 GMT isdst=0 gmtoff=0
# Europe/London  Sun Mar 29 01:00:00 2020 UT = Sun Mar 29 02:00:00 2020 BST isdst=1 gmtoff=3600
# Europe/London  Sun Oct 25 00:59:59 2020 UT = Sun Oct 25 01:59:59 2020 BST isdst=1 gmtoff=3600
# Europe/London  Sun Oct 25 01:00:00 2020 UT = Sun Oct 25 01:00:00 2020 GMT isdst=0 gmtoff=0

def parse_zdump_row(r):
  zone, r = r.split('  ',1)
  if " UT " in r:
    ut, r   = r.split(' = ', 1)
    # parsedate ignores timezone, treats as UTC regardless
    ut = int(calendar.timegm( email.utils.parsedate(ut) ))
  if "isdst" in r and "gmtoff" in r:
    dst =    int( r.split('isdst=')[1][0] )
    gmtoff = int( r.split('gmtoff=')[1] )

  return zone, ut, dst, gmtoff


def encode_transition(t, gmtoff, dst):
  b = bytearray()
  if t<0:
    t=0
  # 32bit unsigned timestamp (valid to year 2106)
  b.append( (t & 0x000000FF) >> 0  )
  b.append( (t & 0x0000FF00) >> 8  )
  b.append( (t & 0x00FF0000) >> 16 )
  b.append( (t & 0xFF000000) >> 24 )

  # 32 bit signed offset
  b.append( (gmtoff & 0x000000FF) >> 0 )
  b.append( (gmtoff & 0x0000FF00) >> 8 )
  b.append( (gmtoff & 0x00FF0000) >> 16 )
  b.append( (gmtoff & 0xFF000000) >> 24 )

  # store dst flag maybe...
  return b

zdumpOverride={
  'Etc/UTC'   :0,
  'Etc/GMT-12':3600*12,
  'Etc/GMT-11':3600*11,
  'Etc/GMT-10':3600*10,
  'Etc/GMT-9' :3600*9,
  'Etc/GMT-8' :3600*8,
  'Etc/GMT-7' :3600*7,
  'Etc/GMT-6' :3600*6,
  'Etc/GMT-5' :3600*5,
  'Etc/GMT-4' :3600*4,
  'Etc/GMT-3' :3600*3,
  'Etc/GMT-2' :3600*2,
  'Etc/GMT-1' :3600*1,
  'Etc/GMT'   :0,
  'Etc/GMT+1' :3600*-1,
  'Etc/GMT+2' :3600*-2,
  'Etc/GMT+3' :3600*-3,
  'Etc/GMT+4' :3600*-4,
  'Etc/GMT+5' :3600*-5,
  'Etc/GMT+6' :3600*-6,
  'Etc/GMT+7' :3600*-7,
  'Etc/GMT+8' :3600*-8,
  'Etc/GMT+9' :3600*-9,
  'Etc/GMT+10':3600*-10,
  'Etc/GMT+11':3600*-11,
  'Etc/GMT+12':3600*-12
}

for zone in zoneNames:
  #category, name = zone.split('/',1)
  print(f"[{zoneNames.index(zone)}/{len(zoneNames)}] {zone}")

  lookupAddr[zone] = output.tell()
  lookupCount[zone] = 0

  if zone in zdumpOverride:
    lookupCount[zone] +=1
    output.write(encode_transition(0, zdumpOverride[zone], 0))
    continue

  rows = subprocess.run(
    [ZDUMP, '-V', zone, '-c','2106'], capture_output=True
    ).stdout.decode('utf-8').split('\n')[:-1]

  if len(rows)==0:
    raise Exception("zdump returned no output")

  for i,j in zip(rows[0::2], rows[1::2]):
    _, utm1, _, _      = parse_zdump_row( i )
    _, ut, dst, gmtoff = parse_zdump_row( j )
    if ut-utm1 != 1:
      raise Exception("Transitions out of sync")
    #print(ut, dst, gmtoff)

    # ignore data from the past, but keep the most recent record so we know
    # what rule is relevant throught the start
    # Also, for zones without DST, all of the data will be from before start
    if ut < start:
      lookupCount[zone]=0
      output.seek(lookupAddr[zone])

    if ut > end:
      continue

    lookupCount[zone] +=1
    output.write(encode_transition(ut, gmtoff, dst))

  if lookupCount[zone]>255:
    raise Exception("More than 255 entries")


#now go back and fill out the addresses
output.seek(startOfNames)

for zone in zoneNames:
  category, name = zone.split('/',1)
  output.write(name.encode())
  # null, 24bit address, 8bit count
  output.write(bytearray([
    0,
    (lookupAddr[zone] & 0x0000FF) >> 0,
    (lookupAddr[zone] & 0x00FF00) >> 8,
    (lookupAddr[zone] & 0xFF0000) >> 16,
    lookupCount[zone]
    ]))

