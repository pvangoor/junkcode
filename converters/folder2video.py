#!/usr/bin/env python3
import argparse
import glob
import cv2
import progressbar

parser = argparse.ArgumentParser(description="Convert a directory of images into a video.")
parser.add_argument("folder", metavar='f', type=str, help="name of the directory containing images.")
parser.add_argument("-r", "--rate", type=float, default=30.0, help="frame rate of the resulting video. Default 30.0")
parser.add_argument("-e", "--extension", type=str, default='png', help="file extension of the images. Default png")
args = parser.parse_args()

folderName = args.folder
if not folderName.endswith('/'):
    folderName = folderName + '/'

images = glob.glob(folderName+"*."+args.extension)
images.sort()

if len(images) == 0:
    print('No images found in "'+folderName+'"')
    exit(-1)

img0 = cv2.imread(images[0])

videoName = folderName[:-1]+'.mkv'
writer = cv2.VideoWriter(videoName, cv2.VideoWriter_fourcc(*"X264"), args.rate, (img0.shape[1], img0.shape[0]))


bar = progressbar.ProgressBar(max_value=len(images))
count = 0
for imageName in images:
    img = cv2.imread(imageName)
    writer.write(img)

    count += 1
    bar.update(count)

bar.finish()
