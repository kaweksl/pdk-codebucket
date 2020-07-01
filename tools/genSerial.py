#!/usr/bin/python
import time
import json

with open('serial.txt') as json_file:
    data = json.load(json_file)
#data['lastsn'] = 0x123456789ABCDFF3
data['lastsn'] = data['lastsn'] + 1

print("0x{:X}".format(data['lastsn']))

with open('serial.txt', 'w') as outfile:
    json.dump(data, outfile, indent=4)
