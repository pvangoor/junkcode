#!/usr/bin/env python3
import argparse
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import progressbar

def update_animation(frame, imu_lines, times, imu_data, ax, frame_rate):
    # global ax

    frame = frame % frames

    time1 = frame / frame_rate
    time0 = max(time1 - args.length, 0.0)

    idx0 = np.argmax(times >= time0)
    idx1 = np.argmax(times >= time1)
    if idx1 <= idx0:
        idx1 = idx0+2

    for i in range(6):
        imu_lines[i].set_data(times[idx0:idx1, 0], imu_data[idx0:idx1, i])
        ax.ravel()[i].set_xlim([times[idx0], times[idx1-1]])

    bar.update(frame)
        



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Animate IMU data over time.")
    parser.add_argument("file", metavar='f', type=str, help="IMU file name.")
    parser.add_argument("--length", type=float, default=5., help="The number of seconds to show. Default 5.0s")
    parser.add_argument("--rate", type=float, default=20., help="The frame rate for the animation. Default 20.0 Hz")
    parser.add_argument("--save", action='store_true', help="Save the animation instead of showing it.")
    args = parser.parse_args()


    with open(args.file, 'r') as f:
        reader = csv.reader(f)
        next(reader)

        # Read the IMU data
        times = []
        imu_data = []
        for row in reader:
            times.append(float(row[0]))
            imu_data.append(np.array([float(r) for r in row[1:7]]))

        imu_data = np.vstack(imu_data)
        gyr_data = imu_data[:, 0:3]
        acc_data = imu_data[:, 3:6]
        times = np.reshape(times, (-1,1))

    fig, ax = plt.subplots(3,2)

    imu_lines = []
    for i in range(6):
        imu_line, = ax.ravel()[i].plot(times, imu_data[:,i], 'b')
        imu_lines.append(imu_line)
    
    ax[0,0].set_ylabel("gyr x")
    ax[1,0].set_ylabel("gyr y")
    ax[2,0].set_ylabel("gyr z")
    ax[0,1].set_ylabel("acc x")
    ax[1,1].set_ylabel("acc y")
    ax[2,1].set_ylabel("acc z")

    total_time = np.max(times) - np.min(times)
    frames = int(total_time * args.rate)

    ani = FuncAnimation(fig, update_animation, frames=frames, interval=1.0 / args.rate, fargs=[imu_lines, times, imu_data, ax, args.rate])
    plt.tight_layout()

    bar = progressbar.ProgressBar(max_value=frames)

    if args.save:
        ani.save('rolling_imu_animation.mp4', fps=50)
    else:
        plt.show()
    
    bar.finish()

