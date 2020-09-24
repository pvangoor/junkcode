#!/usr/bin/env python3
import csv
import argparse
from pylie import SE3
import numpy as np

parser = argparse.ArgumentParser(description="Compute ideal IMU measurements from pose data.")
parser.add_argument("fname", metavar='f', type=str, help="The csv file containing pose data.")
parser.add_argument("--fspec", type=str, default='xw', help="Format of the poses in the file. Default xw.")
parser.add_argument("--step", type=float, default=1.0, help="The time between poses. Default 1.0. Negative values signal that the poses are stored with timestamps before the data and the scale of the time in seconds. e.g. -1e-9 indicates data in nanoseconds.")
parser.add_argument("--startrow", type=int, default=1, help="The row pose data starts. Default 1.")
parser.add_argument("--gravity", type=tuple, default=(0,0,9.81), help="The gravity vector. Default (0,0,9.81).")
args = parser.parse_args()

default_step = args.step
timestamps_available = (args.step <= 0)
if timestamps_available:
    timestamp_scaling = -args.step 

fname = args.fname
oname = fname[:-4] + "_IMU.csv"

gravity = np.reshape(np.array(args.gravity), (3,1))

# Read in the poses
with open(fname, 'r') as f, open(oname, 'w') as outf:
    reader = csv.reader(f)
    writer = csv.writer(outf)

    # Skip rows until the start
    for _ in range(args.startrow):
        next(reader)
    
    # Write the output header
    writer.writerow(["t", "Omega_x (rad/s)", "Omega_y (rad/s)", "Omega_z (rad/s)", "Accel_x (m/s^2)", "Accel_y (m/s^2)", "Accel_z (m/s^2)"])
    
    last_pose = None
    last_vel = None
    last_stamp = 0.0
    # Read the poses to file
    for (line_number,line) in enumerate(reader):
        # Get the timestamp
        if timestamps_available:
            stamp = float(line[0]) * timestamp_scaling
            line = line[1:]
        else:
            stamp = default_step * line_number

        # Get the pose
        pose = SE3.from_list(line, args.fspec)
        
        # Handle the first instance
        if last_pose is None:
            last_pose = pose
            last_stamp = stamp
            continue

        # Compute the pose difference
        dt = stamp - last_stamp
        if dt <= 0.0:
            last_pose = pose
            last_stamp = stamp
            last_vel = None
            continue

        pose_change = last_pose.inv() * pose
        U_vec = pose_change.log() / dt
        Omega = U_vec[0:3,0:1]
        vel = U_vec[3:6,0:1]
        
        # Compute the acceleration
        if last_vel is None:
            last_vel = vel
        accel = (last_pose.R().T @ pose.R() @ vel - last_vel) / dt - last_pose.R().T @ gravity
        
        # Write everything to file
        new_line = [stamp] + Omega.ravel().tolist() + accel.ravel().tolist()
        writer.writerow(new_line)

        # Prepare for the next iteration
        last_pose = pose
        last_stamp = stamp
        last_vel = vel

