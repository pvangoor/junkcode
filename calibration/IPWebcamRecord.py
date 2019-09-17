import cv2
import time
import argparse

parser = argparse.ArgumentParser(description="Capture images from an ip webcam and save to a video file.")
parser.add_argument('address', metavar='a', type=str, help="The address of the ip webcam.")
parser.add_argument('video', metavar='v', type=str, help="The file name of the video to save")

args = parser.parse_args()
website = args.address
videoFname = args.video

cap = cv2.VideoCapture(website)
flag = False
while not flag:
    flag, frame = cap.read()
# out = cv2.VideoWriter(videoFname, cap.get(cv2.CAP_PROP_FOURCC), cap.get(cv2.CAP_PROP_FOURCC))
# out = cv2.VideoWriter(videoFname, cv2.VideoWriter_fourcc('M','J','P','G'), 20, (frame.shape[1], frame.shape[0]))
out = cv2.VideoWriter(videoFname, cv2.VideoWriter_fourcc('H','2','6','4'), 20, (frame.shape[1], frame.shape[0]))

diffAvg = 0.0
diffCount = 0
ct = time.time()

while(1):
    flag, frame = cap.read()

    if flag:
        cv2.imshow('test', frame)
        k = cv2.waitKey(1)

        diff = time.time() - ct
        diffAvg = (diffCount*diffAvg + diff) / (diffCount + 1)
        diffCount += 1
        ct = time.time()

        # print "Average dt:", diffAvg

        out.write(frame)
    
    if k==27:
        break

