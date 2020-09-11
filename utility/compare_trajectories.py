#!/usr/bin/env python3
from pylie import SE3
from pylie import analysis
import csv
import argparse

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser(description="Compare estimated and groundtruth trajectories.")
parser.add_argument("est_poses", metavar='f', type=str, help="The file containing estimated poses.")
parser.add_argument("gt_poses", metavar='g', type=str, help="The file containing groundtruth poses.")
parser.add_argument("--fformat", type=str, default="xq", help="The format of est poses. Default xq")
parser.add_argument("--gformat", type=str, default="xw", help="The format of groundtruth poses. Default xw")
parser.add_argument("--num_frames", type=int, default=100, help="The number of frames used to match poses. Use -1 for full alignment. Default 100.")

args = parser.parse_args()


def readTimedPoses(fname : str, format_spec : str):
    with open(fname, 'r') as file:
        reader = csv.reader(file)
        # Skip header
        next(reader)
        times = []
        poses = []
        for line in reader:
            times.append(float(line[0]))
            poses.append(SE3.from_list(line[1:], format_spec))
    return times, poses

def changePoseTimes(new_times : list, times : list, poses : list):
    # Compute the poses at new_times rather than times
    new_poses = []
    for t in new_times:
        try:
            old_time_index = next(i for i, ti in enumerate(times) if ti >= t)
            new_poses.append(poses[old_time_index])
        except StopIteration:
            new_poses.append( poses[-1] )
    return new_poses
        
print("Reading the estimated poses...")
etimes, fposes = readTimedPoses(args.est_poses, args.fformat)
print("Reading the ground truth poses...")
gtimes, gposes = readTimedPoses(args.gt_poses, args.gformat)
print("Matching the ground truth to the estimated...")
gposes = changePoseTimes(etimes, gtimes, gposes)

if args.num_frames >= 0:
    frame_change_n_times = min(args.num_frames, len(etimes))
    print("Using the first {} positions to align the trajectories.".format(frame_change_n_times))
else:
    frame_change_n_times = len(etimes)
    print("Using all positions to align the trajectories.")
f_positions = np.hstack([pose._x._trans for pose in fposes])
g_positions = np.hstack([pose._x._trans for pose in gposes])
frame_change = analysis.umeyama(f_positions[:,:frame_change_n_times], g_positions[:,:frame_change_n_times])
frame_change = frame_change.to_SE3().inv()
aposes = [(frame_change * pose) for pose in gposes]

n_poses = len(fposes)
rel_fposes = [fposes[i].inv() * fposes[i+1] for i in range(n_poses-1)]
rel_aposes = [aposes[i].inv() * aposes[i+1] for i in range(n_poses-1)]
rel_ftrans = np.hstack([rel_pose._x._trans for rel_pose in rel_fposes])
rel_gtrans = np.hstack([rel_pose._x._trans for rel_pose in rel_aposes])

rel_frot = np.hstack([rel_pose._R.log() for rel_pose in rel_fposes])
rel_grot = np.hstack([rel_pose._R.log() for rel_pose in rel_aposes])

frot_eul = np.hstack([np.reshape(pose._R._rot.as_euler('xyz'), (3,1)) for pose in fposes])
grot_eul = np.hstack([np.reshape(pose._R._rot.as_euler('xyz'), (3,1)) for pose in aposes])
ftrans = np.hstack([np.reshape(pose._x._trans, (3,1)) for pose in fposes])
gtrans = np.hstack([np.reshape(pose._x._trans, (3,1)) for pose in aposes])


# Plot the relative translation and rotation
fig, ax = plt.subplots(3,2)
ax[0,0].plot(etimes[:-1], rel_gtrans[0,:], 'r',
             etimes[:-1], rel_ftrans[0,:], 'r--')
ax[1,0].plot(etimes[:-1], rel_gtrans[1,:], 'g',
             etimes[:-1], rel_ftrans[1,:], 'g--')
ax[2,0].plot(etimes[:-1], rel_gtrans[2,:], 'b',
             etimes[:-1], rel_ftrans[2,:], 'b--')
ax[0,1].plot(etimes[:-1], rel_grot[0,:], 'r',
             etimes[:-1], rel_frot[0,:], 'r--')
ax[1,1].plot(etimes[:-1], rel_grot[1,:], 'g',
             etimes[:-1], rel_frot[1,:], 'g--')
ax[2,1].plot(etimes[:-1], rel_grot[2,:], 'b',
             etimes[:-1], rel_frot[2,:], 'b--')
           
ax[0,0].set_ylabel("V_x")
ax[1,0].set_ylabel("V_y")
ax[2,0].set_ylabel("V_z")
ax[0,0].set_title("Position Change")
ax[2,0].set_xlabel('Time')
ax[0,1].set_ylabel("Omega_x")
ax[1,1].set_ylabel("Omega_y")
ax[2,1].set_ylabel("Omega_z")
ax[0,1].set_title("Attitude Change")
ax[2,1].set_xlabel('Time')
for a in ax.ravel():
    a.set_xlim((etimes[0], etimes[-2]))


# Plot the absolute translation and rotation
fig,ax = plt.subplots(3,2)
ax[0,0].plot(etimes, gtrans[0,:], 'r',
             etimes, ftrans[0,:], 'r--')
ax[1,0].plot(etimes, gtrans[1,:], 'g',
             etimes, ftrans[1,:], 'g--')
ax[2,0].plot(etimes, gtrans[2,:], 'b',
             etimes, ftrans[2,:], 'b--')
ax[0,1].plot(etimes, grot_eul[0,:], 'r',
             etimes, frot_eul[0,:], 'r--')
ax[1,1].plot(etimes, grot_eul[1,:], 'g',
             etimes, frot_eul[1,:], 'g--')
ax[2,1].plot(etimes, grot_eul[2,:], 'b',
             etimes, frot_eul[2,:], 'b--')

ax[0,0].set_ylabel("Pos_x")
ax[1,0].set_ylabel("Pos_y")
ax[2,0].set_ylabel("Pos_z")
ax[0,0].set_title("Position")
ax[2,0].set_xlabel('Time')
ax[0,1].set_ylabel("Eul_x")
ax[1,1].set_ylabel("Eul_y")
ax[2,1].set_ylabel("Eul_z")
ax[0,1].set_title("Attitude")
ax[2,1].set_xlabel('Time')
for a in ax.ravel():
    a.set_xlim((etimes[0], etimes[-1]))
for i in range(3):
    ax[i,1].set_ylim((-np.pi, np.pi))

# Plot the full trajectories (position xy)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(np.hstack([pose._x._trans[0] for pose in aposes]),
        np.hstack([pose._x._trans[1] for pose in aposes]),
        np.hstack([pose._x._trans[2] for pose in aposes]))
ax.plot(np.hstack([pose._x._trans[0] for pose in fposes]),
        np.hstack([pose._x._trans[1] for pose in fposes]),
        np.hstack([pose._x._trans[2] for pose in fposes]))



plt.show()