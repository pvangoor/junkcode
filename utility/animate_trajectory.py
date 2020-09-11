#!/usr/bin/env python3
from pylie import plotting
from pylie import SE3
import argparse
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

parser = argparse.ArgumentParser(description="Animate the trajectory of a pose in a file.")
parser.add_argument("file", metavar='f', type=str, help="Name fo the file with the poses.")
parser.add_argument("--fspec", type=str, default='xw', help="Format of the poses in the file. Default xw.")
parser.add_argument("--scol", type=int, default=1, help="The column where the poses start. Default 1.")
parser.add_argument("--srow", type=int, default=1, help="The row where the poses start. Default 1.")
parser.add_argument("--delim", type=str, default=",", help="The delimiter. Default \",\".")
parser.add_argument("--skip", type=int, default=0, help="The number of poses to skip between frames. Default 0.")

args = parser.parse_args()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


with open(args.file, 'r') as f:
    reader = csv.reader(f, delimiter=args.delim)

    # Ignore the header
    for _ in range(args.srow):
        next(reader)

    # Read the poses and animate trajectory
    trail = np.zeros((3,0))
    counter = -1
    for line in reader:

        counter += 1
        if args.skip == 0 or counter % args.skip != 0:
            continue

        pose = SE3.from_list(line[args.scol:], args.fspec)
        trail = np.hstack((trail,pose.x()))
        # print(pose.q())

        ax.cla()

        ax.plot(trail[0,:], trail[1,:], trail[2,:], "m")
        plotting.plotFrame(pose, '-', ax)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_box_aspect(np.max(trail, axis=1)-np.min(trail,axis=1) + np.ones(3)*2)

        plt.pause(0.02) # 50 Hz

