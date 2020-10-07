import sys, os
import rosbag
import csv

from dvs_msgs.msg import Event, EventArray
from std_msgs.msg import Time

import argparse

parser = argparse.ArgumentParser(description="Convert a csv of events to a rosbag")
parser.add_argument("eventsFile", metavar="f", type=str, help="File containing events.")
parser.add_argument("--delim", default=" ", type=str, help="Delimiter of the events file. Default \" \"")
parser.add_argument("--arraySize", default=200, type=int, help="Size of each event array. Default: 200")
parser.add_argument("--timeScale", default=1.0, type=float, help="Scale of the timestamps. e.g. 1e-9 for nanoseconds. Default: 1.0")
args = parser.parse_args()

fname = args.eventsFile
oname = fname[:fname.rfind(".")] + "_converted.bag"

bag = rosbag.Bag(oname, 'w')

with open(fname) as f:
    reader = csv.reader(f, delimiter=args.delim)
    event_list = []
    for row in reader:
        if len(event_list) >= args.arraySize:
            # Write to rosbag
            eventArray = EventArray()
            eventArray.events = event_list
            # TODO: Finish off eventArray

            # TODO: Does this need a time or anything else?
            bag.write("/dvs/events", eventArray)
            event_list = []

        emsg = Event()
        etime = float(row[0]) * args.timeScale
        emsg.ts = Time(etime)
        emsg.x = int(row[1])
        emsg.x = int(row[2])
        emsg.polarity = bool(row[3])

        event_list.append(emsg)
