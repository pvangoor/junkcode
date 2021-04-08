#!/usr/bin/env python3
from genpy.rostime import Time
import rosbag
import argparse
import csv
import cv2
import cv_bridge

from sensor_msgs.msg import Imu, Image

parser = argparse.ArgumentParser("Convert an ardupilot VIO dataset into a rosbag.")
parser.add_argument("imu", metavar='i', help="The file containing the IMU data.")
parser.add_argument("video", metavar='v', help="The file containing the video data.")
parser.add_argument("stamps", metavar='s', help="The file containing the time stamps of each video frame.")
args = parser.parse_args()

# Open all the files
bag_file = rosbag.Bag("mav_vio.bag", 'w')

print("Writing IMU data...")
with open(args.imu, 'r') as imu_file:
    imu_reader = csv.reader(imu_file)
    next(imu_reader) # skip header
    seq_counter = 1
    for row in imu_reader:
        ts = float(row[0])
        gyr = [float(row[i]) for i in range(1,4)]
        acc = [float(row[i]) for i in range(4,7)]

        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu'
        imu_msg.header.seq = seq_counter
        seq_counter += 1
        imu_msg.header.stamp = Time.from_sec(ts)

        imu_msg.angular_velocity.x = gyr[0]
        imu_msg.angular_velocity.y = gyr[1]
        imu_msg.angular_velocity.z = gyr[2]

        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]

        bag_file.write('/imu', imu_msg, t=imu_msg.header.stamp)
        # bag_file.write('/imu', imu_msg)

print("Writing video data...")
cap = cv2.VideoCapture(args.video)
bridge = cv_bridge.CvBridge()
with open(args.stamps, 'r') as stamps_file:
    stamps_reader = csv.reader(stamps_file)
    next(stamps_reader) # skip header

    for row in stamps_reader:
        ts = float(row[0])
        frame_count = int(row[1])

        ret, frame = cap.read()
        if not ret:
            print("Camera data ended at frame {}.".format(frame_count))
            break

        img_msg = bridge.cv2_to_imgmsg(frame)

        img_msg.header.frame_id = 'cam'
        img_msg.header.seq = frame_count
        img_msg.header.stamp = Time.from_sec(ts)

        bag_file.write('/cam', img_msg, t=img_msg.header.stamp)
        # bag_file.write('/cam', img_msg)
cap.release()

# bag_file.reindex()
bag_file.close()