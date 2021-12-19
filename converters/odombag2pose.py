#!/usr/bin/env python3
import argparse

import rosbag
import progressbar
import csv
from nav_msgs.msg import Odometry


def extract_odom_line(msg: Odometry):
    t = msg.header.stamp.to_sec()
    pose_list = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
    ]
    return [t] + pose_list


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description="Take odometry messages from a rosbag to a file.")
    parser.add_argument('rosbag', metavar='b', type=str,
                        help="The file name of the rosbag to convert.")
    parser.add_argument('--topic', type=str, default="/rovio/odometry",
                        help="The topic to get the odometry messages from.")
    args = parser.parse_args()

    bagFname = args.rosbag
    posesFname = bagFname[:bagFname.rfind(".")] + "_poses.csv"

    with rosbag.Bag(bagFname, 'r') as bag, open(posesFname, 'w') as posesFile:
        posesWriter = csv.writer(posesFile)

        # Write the header
        posesWriter.writerow(
            "time [s], qw, qx, qy, qz, px [m], py [m], pz [m]".split(',')
        )

        progBar = progressbar.ProgressBar(
            max_value=bag.get_message_count(args.topic))
        messageCount = 0

        for topic, msg, t in bag.read_messages():
            if topic == args.topic:
                posesWriter.writerow(extract_odom_line(msg))

                messageCount += 1
                progBar.update(messageCount)
        progBar.finish()
