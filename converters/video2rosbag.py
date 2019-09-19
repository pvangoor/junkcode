import argparse
import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image
import cv_bridge

parser = argparse.ArgumentParser(description="Convert a video file to a rosbag.")
parser.add_argument('video', metavar='v', type=str, help="The file name of the video to convert")
args = parser.parse_args()

bridge = cv_bridge.CvBridge()



videoFname = args.video
bagFname = videoFname[:-4] + ".bag"
f = open(bagFname, 'w')
bag = rosbag.Bag(f,'w')

cap = cv2.VideoCapture(videoFname)
flag, frame = cap.read()

frame_rate = cap.get(cv2.CAP_PROP_FPS)
time = 1/frame_rate
last_time = 0

count = 0
while (flag):
    msg = bridge.cv2_to_imgmsg(frame, "passthrough")
    ros_time = rospy.Time(time)
    msg.header.stamp = ros_time

    if (time - last_time >= 1.0/4.0):
        bag.write('/cam0/image_raw/', msg, ros_time)
        last_time = time
        print count
    count += 1

    cv2.imshow('debug', frame)
    cv2.waitKey(1)


    flag, frame = cap.read()
    time += 1/frame_rate

bag.close()

    



