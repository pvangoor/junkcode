import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import cv_bridge
import csv
import numpy as np

cam_delay = -0.162
frame_rate = 28.886794
last_time_cam = 0
time_cam_base = 595.490854 + cam_delay
time_cam = time_cam_base

f = open('data/calib_vio.bag', 'w')
bag = rosbag.Bag(f,'w')

bridge = cv_bridge.CvBridge()

cap = cv2.VideoCapture("data/calib2.mp4")
flag, frame = cap.read()

# do images
count = 0
while (flag):
    msg = bridge.cv2_to_imgmsg(frame, "passthrough")
    ros_time = rospy.Time(time_cam)
    msg.header.stamp = ros_time

    if (time_cam - last_time_cam >= 1.0/20.0):
        bag.write('/cam0/image_raw/', msg, ros_time)
        last_time_cam = time_cam
        print count
    time_cam += 1/frame_rate
    flag, frame = cap.read()
    count += 1

# do imu
# nkf1 = csv.reader(open("data/nkf1.csv",'r'))
# nkf2 = csv.reader(open("data/nkf2.csv",'r'))
imu = csv.reader(open("data/imu.csv", 'r'))

count = 0
for row in imu:
    if count == 0:
        count += 1
        continue
    
    timestamp = float(row[0])

    if (timestamp < time_cam_base - 5.0):
        continue
    
    if (timestamp > time_cam + 5.0):
        break

    gxyz = np.array([float(a) for a in row[2:5]])
    axyz = np.array([float(a) for a in row[5:8]])

    # print gxyz
    # print axyz

    rosimu = Imu()
    rosimu.header.stamp = rospy.Time(timestamp)
    rosimu.angular_velocity.x = float(gxyz[0])
    rosimu.angular_velocity.y = float(gxyz[1])
    rosimu.angular_velocity.z = float(gxyz[2])
    rosimu.linear_acceleration.x = float(axyz[0])
    rosimu.linear_acceleration.y = float(axyz[1])
    rosimu.linear_acceleration.z = float(axyz[2])
    
    bag.write('/imu0', rosimu, rospy.Time(timestamp))

    count += 1
    print count



# cv2.imshow('debug', frame)
# cv2.waitKey(0)