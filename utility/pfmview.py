#!/usr/bin/env python3
import cv2
import numpy as np
import argparse

parser = argparse.ArgumentParser(description="View a pfm file by normalizing.")
parser.add_argument("fname", metavar='f', type=str, help="The file name to view.")
args = parser.parse_args()

fname = args.fname
assert (fname.endswith(".pfm")), "The file is not a pfm."

img = cv2.imread(fname, cv2.IMREAD_UNCHANGED)
# Normalise the image
max_val = np.max(img)
img = img / max_val * 255
img = np.round(img)
img = img.astype(np.uint8)

cv2.imshow(fname, img)
cv2.waitKey(0)
