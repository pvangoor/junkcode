#!/usr/bin/env python3
from pylie import plotting
from pylie import SO3
import argparse
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def update_animation(frame, attitudes, horizon_artist):
    if frame < len(attitudes):
        horizon_artist.set_attitude_data(attitudes[frame])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Animate the horizon of an attitude in a file.")
    parser.add_argument("file", metavar='f', type=str, help="Name of the file with the attitudes.")
    parser.add_argument("--fspec", type=str, default='w', help="Format of the attitudes in the file. Default xw.")
    parser.add_argument("--scol", type=int, default=4, help="The column where the attitudes start. Default 4.")
    parser.add_argument("--srow", type=int, default=1, help="The row where the attitudes start. Default 1.")
    parser.add_argument("--delim", type=str, default=",", help="The delimiter. Default \",\".")
    parser.add_argument("--skip", type=int, default=0, help="The number of attitudes to skip between frames of animation. Default 0.")

    args = parser.parse_args()


    with open(args.file, 'r') as f:
        reader = csv.reader(f, delimiter=args.delim)

        # Ignore the header
        for _ in range(args.srow):
            next(reader)

        # Read the attitudes and animate trajectory
        attitudes = []
        counter = -1
        for line in reader:

            counter += 1
            if args.skip != 0 and counter % args.skip != 0:
                continue

            attitude = SO3.from_list(line[args.scol:], args.fspec)
            attitudes.append(attitude)

    fig = plt.figure()
    ax = fig.add_subplot()
    attitudes = [ attitudes[0].inv() * att for att in attitudes]
    horizon_artist = plotting.plotArtificialHorizon(attitudes[0])

    ani = FuncAnimation(fig, update_animation, frames=len(attitudes)+500, interval=20, fargs=[attitudes, horizon_artist])
    plt.show()

