#!/usr/bin/env python3

import csv, time, collections

start = int(time.mktime(time.strptime("1 Jan 2020", "%d %b %Y")))
end = int(time.mktime(time.strptime("1 Jan 2100", "%d %b %Y")))

zones = collections.OrderedDict()
zoneIDs = {}
lookupAddr = {}
lookupCount = {}

output = open("tzrules.bin", "wb")
output.write(b'MTZ') # header
output.write(b'\x01')      # version number
output.write(b'\x08')      # row length for fixed data


with open('zone.csv') as csvfile:
  reader = csv.reader(csvfile, delimiter=',', quotechar='"')
  for row in reader:
    category = row[2].split('/')[0]
    name = '/'.join(row[2].split('/')[1:])

    if category not in zones:
      zones[category]=[]
    zones[category].append({'name':name, 'key':int(row[0])})
    zoneIDs[int(row[0])] = {'category':category, 'name':name}

# Etc zones correspond to oceans and are not included in the timezonedb
zones['Etc']=[
  {'name':'UTC'   ,'key':1001,'offset':0},
  {'name':'GMT-12','key':1002,'offset':3600*12},
  {'name':'GMT-11','key':1003,'offset':3600*11},
  {'name':'GMT-10','key':1004,'offset':3600*10},
  {'name':'GMT-9' ,'key':1005,'offset':3600*9},
  {'name':'GMT-8' ,'key':1006,'offset':3600*8},
  {'name':'GMT-7' ,'key':1007,'offset':3600*7},
  {'name':'GMT-6' ,'key':1008,'offset':3600*6},
  {'name':'GMT-5' ,'key':1009,'offset':3600*5},
  {'name':'GMT-4' ,'key':1010,'offset':3600*4},
  {'name':'GMT-3' ,'key':1011,'offset':3600*3},
  {'name':'GMT-2' ,'key':1012,'offset':3600*2},
  {'name':'GMT-1' ,'key':1013,'offset':3600*1},
  {'name':'GMT'   ,'key':1014,'offset':0},
  {'name':'GMT+1' ,'key':1015,'offset':3600*-1},
  {'name':'GMT+2' ,'key':1016,'offset':3600*-2},
  {'name':'GMT+3' ,'key':1017,'offset':3600*-3},
  {'name':'GMT+4' ,'key':1018,'offset':3600*-4},
  {'name':'GMT+5' ,'key':1019,'offset':3600*-5},
  {'name':'GMT+6' ,'key':1020,'offset':3600*-6},
  {'name':'GMT+7' ,'key':1021,'offset':3600*-7},
  {'name':'GMT+8' ,'key':1022,'offset':3600*-8},
  {'name':'GMT+9' ,'key':1023,'offset':3600*-9},
  {'name':'GMT+10','key':1024,'offset':3600*-10},
  {'name':'GMT+11','key':1025,'offset':3600*-11},
  {'name':'GMT+12','key':1026,'offset':3600*-12},
]

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
    output.write(z['name'].encode())
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



# We assume the data timezone.csv file is sorted, ascending

key=0
lasttime = int(-1e12) # just for confirming data is in order
dataset = []



def row_to_binary(row):
  b = bytearray()
  t = row[2]
  if t<0:
    t=0
  # 32bit unsigned timestamp (valid to year 2106)
  b.append( (t & 0x000000FF) >> 0  )
  b.append( (t & 0x0000FF00) >> 8  )
  b.append( (t & 0x00FF0000) >> 16 )
  b.append( (t & 0xFF000000) >> 24 )

  # 32 bit signed offset
  b.append( (row[3] & 0x000000FF) >> 0 )
  b.append( (row[3] & 0x0000FF00) >> 8 )
  b.append( (row[3] & 0x00FF0000) >> 16 )
  b.append( (row[3] & 0xFF000000) >> 24 )

  # store name and dst flag maybe...
  return b

def write_data():
  lookupAddr[key] = output.tell()
  lookupCount[key] = 0
  for row in dataset:
    lookupCount[key] +=1
    output.write(row_to_binary(row))



with open('timezone.csv') as csvfile:
  reader = csv.reader(csvfile, delimiter=',', quotechar='"')
  for row in reader:
    row[0] = int(row[0])
    row[2] = int(row[2])
    row[3] = int(row[3])

    if row[0] != key: # end of data for key
      if key!=0: write_data()
      if row[0]!=key+1:
        raise Exception("Missing key or data out-of-order")
      key=row[0]
      lasttime = int(-1e12)
      dataset=[row]
      continue

    time = row[2]
    offset = row[3]
    if (time < lasttime):
      raise Exception("Data out of order!")
    lasttime = time

    if (time > dataset[0][2] and time < start):
       # Ignore data from the past, but keep the most recent record so we know
       # what rule is relevant through the start
       # Also, for zones without DST, all of the data will be from before start
       dataset[0] = row
       continue

    if (time > end):
      continue

    dataset.append(row)


write_data() # trigger again for very last dataset


# run an extra loop to add in the Etc zones
for e in zones['Etc']:
  key = e['key']
  dataset = [[ e['key'],'',0, e['offset'] ]]
  write_data()
  


#now go back and fill out the addresses
output.seek(startOfNames)

for category in zones:
  for z in zones[category]:
    output.write(z['name'].encode())
    # null, 24bit address, 8bit count
    output.write(bytearray([
      0,
      (lookupAddr[z['key']] & 0x0000FF) >> 0,
      (lookupAddr[z['key']] & 0x00FF00) >> 8,
      (lookupAddr[z['key']] & 0xFF0000) >> 16,
      lookupCount[z['key']]
      ]))

