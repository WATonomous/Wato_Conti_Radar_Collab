#!/usr/bin/env python

import rosbag
import sys

types = {}

for bagname in sys.argv[1:]:
    bag = rosbag.Bag(bagname)
    for topic, msg, t in bag.read_messages():
        if not msg._type in types:
            types[msg._type] = msg._full_text


for t in types:
    print ("Message type:", t)
    print ("Message text:")
    print (types[t])
    print ()