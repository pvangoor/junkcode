'''
Video and IMU to ROSbag Converter
Usage: video_imu2rosbag.py -i <imu> -v <video> -t <time"
Arguments:
 -i, --imu          The file containing timestamped IMU data in csv format.
 -v, --video        The video file to be used.
 -t, --time         The starting time of the video w.r.t the IMU data in seconds. Default: 0
 -b, --bag          The output bag file name. Default: output.bag
 -s, --scale        The scale of the video output. e.g. 0.5 will halve the resolution. Default: 1

Flags:
 -h                 help
'''     

import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import cv_bridge
import csv
import numpy as np
import progressbar

import getopt
import sys

def usage():
    print(sys.exit(__doc__))

def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1

try:
    opts, args = getopt.getopt(sys.argv[1:], "i:v:t:bs", ["imu=", "video=", "time=", "bag=", "scale="])
except getopt.GetoptError as err:
    print(str(err))
    usage()

bag_fname = "output.bag"
scale = 0.5
time_offset = 0

for opt, arg in opts:
    if opt == '-h':
        usage()

    elif opt in ("-i", "--imu"):
        imu_fname = arg

    elif opt in ("-v", "--video"):
        video_fname = arg
    
    elif opt in ("-t", "--time"):
        try:
            time_offset = float(arg)
        except ValueError:
            print("Time must be provided as a number.")
            sys.exit(1)
    
    elif opt in ("-s", "--scale"):
        try:
            scale = float(arg)
        except ValueError:
            print("Scale must be provided as a number.")
            print(opts)
            sys.exit(1)
    
    elif opt in ("-b", "--bag"):
        bag_fname = arg
        if not bag_fname.endswith(".bag"):
            bag_fname = bag_fname + ".bag"


# Open the bag file
bag_file = open(bag_fname, 'w')
bag = rosbag.Bag(bag_file,'w')


# Convert the IMU data
imu_file_length = file_len(imu_fname)
imu_file = open(imu_fname, 'r')
imu_reader = csv.reader(imu_file)

imu_start = -1

print("Processing IMU...")
progBar = progressbar.ProgressBar(max_value=imu_file_length)
count = 0
for row in imu_reader:
    # Skip the header
    if count == 0:
        count += 1
        continue

    timestamp = float(row[0])
    gyr_xyz = [float(a) for a in row[1:4]]
    acc_xyz = [float(a) for a in row[4:7]]

    if imu_start < 0:
        imu_start = timestamp

    rosimu = Imu()
    rosimu.header.stamp = rospy.Time(timestamp)
    rosimu.angular_velocity.x = gyr_xyz[0]
    rosimu.angular_velocity.y = gyr_xyz[1]
    rosimu.angular_velocity.z = gyr_xyz[2]
    rosimu.linear_acceleration.x = acc_xyz[0]
    rosimu.linear_acceleration.y = acc_xyz[1]
    rosimu.linear_acceleration.z = acc_xyz[2]
    
    bag.write('/imu0', rosimu, rospy.Time(timestamp))

    count += 1
    progBar.update(count)

imu_file.close()


# Convert the video
bridge = cv_bridge.CvBridge()
cap = cv2.VideoCapture(video_fname)
total_num_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
frame_rate = cap.get(cv2.CAP_PROP_FPS)

frame_time = time_offset + imu_start
flag, frame = cap.read()

print("Processing video...")
progBar = progressbar.ProgressBar(max_value=total_num_frames)
frame_count = 0
last_frame_time = -1e8
while (flag):
    # Write frames at 20Hz only
    if (frame_time - last_frame_time) >= 1.0/20.0:
        height, width = frame.shape[:2]
        frame = cv2.resize(frame,(int(width*scale), int(height*scale)), interpolation = cv2.INTER_AREA)
        msg = bridge.cv2_to_imgmsg(frame, "passthrough")
        ros_time = rospy.Time(frame_time)
        msg.header.stamp = ros_time

        bag.write('/cam0/image_raw/', msg, ros_time)

        last_frame_time = frame_time

    frame_time += 1/frame_rate
    flag, frame = cap.read()
    frame_count += 1
    progBar.update(frame_count)

cap.release()

bag.close()
bag_file.close()