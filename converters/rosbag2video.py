import argparse
import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import progressbar

parser = argparse.ArgumentParser(description="Convert a rosbag to a video file.")
parser.add_argument('rosbag', metavar='b', type=str, help="The file name of the rosbag to convert.")
parser.add_argument('--topic', type=str, default="/cam0/image_raw", help="The topic to get frames from.")
parser.add_argument('--framerate', type=int, default=20, help="The frame rate of the resulting video")
args = parser.parse_args()


bagFname = args.rosbag
videoTopic = args.topic
frame_rate = args.framerate


videoFname = bagFname[:-4] + ".mkv"
f = open(bagFname, 'r')
bag = rosbag.Bag(f,'r')


bridge = cv_bridge.CvBridge()
initialised = False

messageNumber = bag.get_message_count()
progBar = progressbar.ProgressBar(max_value=messageNumber)
messageCount = 0

for topic, msg, t in bag.read_messages():
    if topic == videoTopic:
        cv_img = bridge.imgmsg_to_cv2(msg)

        if not initialised:
            out = cv2.VideoWriter(videoFname, cv2.VideoWriter_fourcc('H','2','6','4'), frame_rate, (cv_img.shape[1], cv_img.shape[0]), isColor=(len(cv_img.shape)>2))
            initialised = True
        
        out.write(cv_img)

    messageCount += 1
    progBar.update(messageCount)


bag.close()
if initialised:
    out.release()
else:
    print("Unable to find any messages under the given topic.")

