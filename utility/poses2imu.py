#!/usr/bin/env python3
import csv
import argparse
from pylie import SE3, SO3
import numpy as np

parser = argparse.ArgumentParser(description="Compute ideal IMU measurements from pose data.")
parser.add_argument("fname", metavar='f', type=str, help="The csv file containing pose data.")
parser.add_argument("--fspec", type=str, default='xw', help="Format of the poses in the file. Default xw.")
parser.add_argument("--step", type=float, default=1.0, help="The time between poses. Default 1.0. Negative values signal that the poses are stored with timestamps before the data and the scale of the time in seconds. e.g. -1e-9 indicates data in nanoseconds.")
parser.add_argument("--startrow", type=int, default=1, help="The row pose data starts. Default 1.")
parser.add_argument("--gravity", type=tuple, default=(0,0,-9.81), help="The gravity vector. Default (0,0,-9.81).")
parser.add_argument("--angbias", type=float, default=0.0, help="The angular velocity bias scale. Default 0.0.")
parser.add_argument("--linbias", type=float, default=0.0, help="The linear acceleration bias scale. Default 0.0.")
args = parser.parse_args()

default_step = args.step
timestamps_available = (args.step <= 0)
if timestamps_available:
    timestamp_scaling = -args.step 

fname = args.fname
oname = fname[:-4] + "_IMU.csv"

gravity = np.reshape(np.array(args.gravity), (3,1))

# Rewrite here

# Get the poses and stamps
with open(fname, 'r') as f:
    reader = csv.reader(f)

    # Skip rows until the start
    for _ in range(args.startrow):
        next(reader)
    
    # Read the poses and stamps
    poses = []
    stamps = []
    for (line_number,line) in enumerate(reader):
        # Get the timestamp
        if timestamps_available:
            stamp = float(line[0]) * timestamp_scaling
            line = line[1:]
        else:
            stamp = default_step * line_number

        # Get the pose
        pose = SE3.from_list(line, args.fspec)

        poses.append(pose)
        stamps.append(stamp)

# Differentiate to get Omega and V
lin_vels = []
ang_vels = []
N = len(poses)
for k in range(N):
    if k >= N-1:
        pose_vel = np.zeros((6,1))
    else:
        dt = stamps[k+1] - stamps[k]
        if dt > 0:
            pose_vel = (poses[k].inv() * poses[k+1]).log() / dt
        else:
            pose_vel = np.zeros((6,1))
    ang_vels.append(pose_vel[0:3,0:1])
    lin_vels.append(pose_vel[3:6,0:1])

# Differentiate again to get Accel
lin_accels = []
for k in range(N):
    if k >= N-2:
        accel = np.zeros((3,1))
    else:
        dt = stamps[k+1] - stamps[k]
        if dt > 0:
            accel = (lin_vels[k+1] - lin_vels[k]) / dt + SO3.skew(ang_vels[k]) @ lin_vels[k] - poses[k].R().as_matrix().T @ gravity
        else:
            accel = SO3.skew(ang_vels[k]) @ lin_vels[k] - poses[k].R().as_matrix().T @ gravity
    lin_accels.append(accel)

# Choose biases
ang_bias = np.random.randn(3,1) * args.angbias
lin_bias = np.random.randn(3,1) * args.linbias

print("The angular velocity bias is\n{}".format(ang_bias))
print("The linear acceleration bias is\n{}".format(lin_bias))

# Write to file
with open(oname, 'w') as outf:
    writer = csv.writer(outf)
    
    # Write the output header
    headerrow = ["t", "Omega_x (rad/s)", "Omega_y (rad/s)", "Omega_z (rad/s)", "Accel_x (m/s^2)", "Accel_y (m/s^2)", "Accel_z (m/s^2)"]
    headerrow += ["bias_Omega_x", "bias_Omega_y", "bias_Omega_z", "bias_accel_x", "bias_accel_y", "bias_accel_z"]
    writer.writerow(headerrow)

    
    # Write the measurements to file
    for k in range(N):
        omega = ang_vels[k] + ang_bias
        accel = lin_accels[k] + lin_bias
        row = [stamps[k]] + omega.ravel().tolist() + accel.ravel().tolist()

        if k == 0:
            row += ang_bias.ravel().tolist() + lin_bias.ravel().tolist()
        elif k == 1:
            row += ["..."] * 6

        writer.writerow(row)