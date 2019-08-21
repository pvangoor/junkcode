import sys, os
import rosbag

import cv2
import cv_bridge
import progressbar

argc = len(sys.argv)
if (argc == 1):
    bagname = 'calib_texture_50.bag'
elif (argc == 2):
    bagname = sys.argv[1]

bag = rosbag.Bag(bagname, 'r')
messageNumber = bag.get_message_count()


bridge = cv_bridge.CvBridge()

# Prepare output directory
assert(bagname.endswith(".bag"))
outputdir = bagname[:-4]
if not os.path.exists(outputdir):
    os.makedirs(outputdir)

imagetimefname = outputdir+"/"+"images.txt"
imagetimefile = open(imagetimefname, 'w')
eventFname = outputdir+"/"+"events.txt"
eventFile = open(eventFname, 'w')
imagedir = outputdir+"/"+"images"
if not os.path.exists(imagedir):
    os.makedirs(imagedir)

imageCounter = 0
msgCounter = 0
progBar = progressbar.ProgressBar(max_value=messageNumber)
for topic, msg, t in bag.read_messages():
    if topic == "/dvs/image_raw":
        img = bridge.imgmsg_to_cv2(msg)
        imageCounter += 1
        imagefilename = "images/{:08d}.png".format(imageCounter)
        timestamp = "{:d}.{:09d}".format(msg.header.stamp.secs,msg.header.stamp.nsecs)
        
        cv2.imwrite(outputdir+"/"+imagefilename, img)

        line = timestamp + " " + imagefilename + "\n"
        imagetimefile.writelines(line)

    elif topic == "/dvs/events":
        for emsg in msg.events:
            timestamp = "{:d}.{:09d}".format(emsg.ts.secs,emsg.ts.nsecs)
            eline = "{} {:d} {:d} {:d}\n".format(timestamp, emsg.x, emsg.y, emsg.polarity)
            eventFile.writelines(eline)

    else:
        print topic
    
    msgCounter += 1
    progBar.update(msgCounter)

progBar.update(messageNumber)