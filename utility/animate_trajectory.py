#!/usr/bin/env python3
from pylie import plotting
from pylie import SE3
import argparse
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np

def update_animation(frame, trail_line, frame_artist, poses, ax):
    # global ax
    if frame < len(poses):
        pose = poses[frame]
        trail_data = list(trail_line.get_data_3d())
        trail_data[0] = np.append(trail_data[0], pose.x().x()[0,0])
        trail_data[1] = np.append(trail_data[1], pose.x().x()[1,0])
        trail_data[2] = np.append(trail_data[2], pose.x().x()[2,0])
        trail_line.set_data_3d(*trail_data)
        
        frame_artist.set_pose_data(pose)

        
        





if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Animate the trajectory of a pose in a file.")
    parser.add_argument("file", metavar='f', type=str, help="Name fo the file with the poses.")
    parser.add_argument("--fspec", type=str, default='xw', help="Format of the poses in the file. Default xw.")
    parser.add_argument("--scol", type=int, default=1, help="The column where the poses start. Default 1.")
    parser.add_argument("--srow", type=int, default=1, help="The row where the poses start. Default 1.")
    parser.add_argument("--delim", type=str, default=",", help="The delimiter. Default \",\".")
    parser.add_argument("--skip", type=int, default=0, help="The number of poses to skip between frames. Default 0.")

    args = parser.parse_args()


    with open(args.file, 'r') as f:
        reader = csv.reader(f, delimiter=args.delim)

        # Ignore the header
        for _ in range(args.srow):
            next(reader)

        # Read the poses and animate trajectory
        poses = []
        counter = -1
        for line in reader:

            counter += 1
            if args.skip != 0 and counter % args.skip != 0:
                continue

            pose = SE3.from_list(line[args.scol:], args.fspec)
            poses.append(pose)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    trail = np.hstack([pose.x().x() for pose in poses])
    trail_line, = ax.plot(trail[0,:], trail[1,:], trail[2,:], 'm')
    ax.set_box_aspect(np.max(trail, axis=1)-np.min(trail,axis=1) + np.ones(3)*2)
    trail_line.set_data_3d([], [], [])
    frame_artist = plotting.plotFrame(pose, '-', ax)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")



    ani = FuncAnimation(fig, update_animation, frames=len(poses)+500, interval=20, fargs=[trail_line, frame_artist, poses, ax])
    plt.show()

