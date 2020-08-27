#!/usr/bin/env python3
import argparse
import rosbag
import csv
import numpy as np

import progressbar

def quat2rod(q):
    print(q[3])
    rod = [s / q[3] for s in q[:3]]
    return rod


parser = argparse.ArgumentParser(description="Convert a vicon rosbag to a csv file.")
parser.add_argument("rosbag", metavar="b", type=str, nargs=1, help="The name of the rosbag file to be converted.")
args = parser.parse_args()

bagname = args.rosbag[0]
bag = rosbag.Bag(bagname, 'r')
messageNumber = bag.get_message_count()

# Prepare output file
assert(bagname.endswith(".bag"))
outputFname = bagname[:-4] + ".csv"
outputFile = open(outputFname, 'w')
outputWriter = csv.writer(outputFile)

header_row = ["time", "tx", "ty", "tz", "rx", "ry", "rz"]
outputWriter.writerow(header_row)

msgCounter = 0
progBar = progressbar.ProgressBar(max_value=messageNumber)
for topic, msg, t in bag.read_messages():
    if topic.startswith("/vicon/"):
        timestamp = 1.0*msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs
        translation = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
        rotation = quat2rod([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])


        row = [timestamp] + translation + rotation
        outputWriter.writerow(row)
    
    msgCounter += 1
    progBar.update(msgCounter)

progBar.update(messageNumber)

outputFile.close()