#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
try:
    from mpldatacursor import datacursor
except ImportError:
    print("mpldatacursor not found.")
import argparse

parser = argparse.ArgumentParser(description="View an image file through pyplot.")
parser.add_argument("fname", metavar='f', type=str, help="The file to view.")
args = parser.parse_args()


img = plt.imread(args.fname)*255
img = img.astype(np.uint8) 

plt.imshow(img)
datacursor()
plt.show()
