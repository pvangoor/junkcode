#!/usr/bin/env python3
import argparse
import cv2
import os
import progressbar

parser = argparse.ArgumentParser(description="Convert a video file to a folder of images.")
parser.add_argument('video', metavar='v', type=str, help="The file name of the video to convert")
args = parser.parse_args()


videoFname = args.video
folderName = videoFname[:-4]
if not os.path.exists(folderName):
    os.makedirs(folderName)

cap = cv2.VideoCapture(videoFname)
flag, frame = cap.read()

# frame_rate = cap.get(cv2.CAP_PROP_FPS)
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
# time = 1/frame_rate
# last_time = 0

if frame_count <= 0:
    print("Frame count unknown. Using 1000.")
    frame_count = 1000

digits = str(len(str(frame_count)))
count = 0
progBar = progressbar.ProgressBar(max_value=frame_count)
while (flag):
    imgFname = ("{:0>"+str(digits)+"d}.png").format(count)
    imgFname = folderName+"/"+imgFname

    cv2.imwrite(imgFname, frame)

    count += 1
    progBar.update(count)


    flag, frame = cap.read()




