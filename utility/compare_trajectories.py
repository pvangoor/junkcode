#!/usr/bin/env python3
from os import error
from pylie import SE3, Trajectory
from pylie import analysis
import csv
import argparse
import yaml

import numpy as np
import matplotlib.pyplot as plt


def read_trajectory(fname: str, format_spec: str, min_time: float = -1.0) -> Trajectory:
    times = []
    poses = []

    scol = 0
    if format_spec[0].isdigit():
        scol = int(format_spec[0])
        format_spec = format_spec[1:]

    with open(fname, 'r') as file:
        reader = csv.reader(file)
        # Skip header
        next(reader)
        for line in reader:
            t = float(line[scol+0])
            if t < 0:
                continue
            if min_time > 0 and t < min_time:
                continue

            pose = SE3.from_list(line[scol+1:], format_spec)
            times.append(t)
            poses.append(pose)

    return Trajectory(poses, times)


def computeStatistics(values: list) -> dict:
    # Compute the statistics (mean, std, median, min, max) from a tuple of values
    stats = {}
    stats["rmse"] = float(np.sqrt(np.mean(values**2)))
    stats["mean"] = float(np.mean(values))
    stats["std"] = float(np.std(values))
    stats["med"] = float(np.median(values))
    stats["min"] = float(np.min(values))
    stats["max"] = float(np.max(values))

    return stats


def statString(stats: dict):
    result = ""
    for key, val in stats.items():
        result += "{:>6s}: {:<.4f}\n".format(key, val)
    return result


def plotStatLines(ax, times, stats):
    tdiff = [times[0], times[-1]]
    ax.plot(tdiff, 2*[stats["mean"]], 'r--')
    ax.plot(tdiff, 2*[stats["mean"]+stats["std"]], 'g--')
    ax.plot(tdiff, 2*[stats["mean"]-stats["std"]], 'g--')
    ax.plot(tdiff, 2*[stats["med"]], 'b--')

def get_traj_velocities(traj):
    traj_vel = np.hstack(traj.get_velocities())
    traj_vel[0:3, :] = traj_vel[0:3, :] * 180.0 / np.pi
    return traj_vel

def compute_errors(traj1: Trajectory, traj2: Trajectory, traj1_vel, traj2_vel):
    # Absolute position
    pos_err_abs = np.hstack([P1.x().as_vector() - P2.x().as_vector()
                            for P1, P2 in zip(traj1.get_elements(), traj2.get_elements())])
    pos_err_abs = np.linalg.norm(pos_err_abs, axis=0)

    att_err_abs = np.hstack([(P1.R().inv() * P2.R()).log()
                            for P1, P2 in zip(traj1.get_elements(), traj2.get_elements())])
    att_err_abs = np.linalg.norm(att_err_abs, axis=0) * 180.0 / np.pi

    pos_err_rel = np.linalg.norm(traj1_vel[3:6, :] - traj2_vel[3:6, :], axis=0)
    att_err_rel = np.linalg.norm(traj1_vel[0:3, :] - traj2_vel[0:3, :], axis=0) * 180.0 / np.pi

    return pos_err_abs, att_err_abs, pos_err_rel, att_err_rel

def gather_error_stats(pos_err_abs, att_err_abs, pos_err_rel, att_err_rel):
    error_dict = {}
    error_dict["absolute"] = {}
    error_dict["absolute"]["position"] = computeStatistics(pos_err_abs)
    error_dict["absolute"]["attitude"] = computeStatistics(att_err_abs)

    error_dict["relative"] = {}
    error_dict["relative"]["position"] = computeStatistics(pos_err_rel)
    error_dict["relative"]["attitude"] = computeStatistics(att_err_rel)

    return error_dict


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Compare estimated and groundtruth trajectories.")
    parser.add_argument("est_poses", metavar='e', type=str,
                        help="The file containing estimated poses.")
    parser.add_argument("gt_poses", metavar='g', type=str,
                        help="The file containing groundtruth poses.")
    parser.add_argument("--eformat", type=str, default="0xw",
                        help="The format of estimated poses. Default 0xw")
    parser.add_argument("--gformat", type=str, default="0xw",
                        help="The format of groundtruth poses. Default 0xw")
    parser.add_argument("--noplot", action='store_true',
                        help="Set to true to suppress plot output.")
    parser.add_argument("--start", type=float, default=0.0,
                        help="Time (from first data) at which to start the comparison. Default 0.0")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="File to write error statistics to. Defalt None.")

    args = parser.parse_args()

    print("Reading the pose files...")
    tru_traj = read_trajectory(args.gt_poses, args.gformat)
    min_time = tru_traj.begin_time() + args.start
    est_traj = read_trajectory(args.est_poses, args.eformat, min_time)

    print("Aligning trajectories...")
    begin_time = max(tru_traj.begin_time(), est_traj.begin_time())
    end_time = min(tru_traj.end_time(), est_traj.end_time())
    tru_traj.truncate(begin_time, end_time)
    est_traj.truncate(begin_time, end_time)
    tru_traj = tru_traj[est_traj.get_times()]

    est_traj, align_params = analysis.align_trajectory(
        est_traj, tru_traj, ret_params=True)

    if len(est_traj) < 2 or len(tru_traj) < 2:
        raise ValueError("No valid poses available.")

    print("Computing errors...")
    est_vel = get_traj_velocities(est_traj)
    tru_vel = get_traj_velocities(tru_traj)
    pos_err_abs, att_err_abs, pos_err_rel, att_err_rel = compute_errors(tru_traj, est_traj, tru_vel, est_vel)
    error_stats = gather_error_stats(pos_err_abs, att_err_abs, pos_err_rel, att_err_rel)
    if args.output:
        with open(args.output, 'w') as f:
            yaml.safe_dump(error_stats, f)
    else:
        print(yaml.safe_dump(error_stats))

    if not args.noplot:
        all_times = tru_traj.get_times()
        # Plot the relative and absolute errors over time
        fig, ax = plt.subplots(2,2)
        error_line_width = 0.2
        ax[0,0].plot(all_times[:-1], pos_err_rel, 'm-', linewidth=error_line_width)
        plotStatLines(ax[0,0], all_times, error_stats["relative"]["position"])
        ax[1,0].plot(all_times[:-1], att_err_rel, 'm-', linewidth=error_line_width)
        plotStatLines(ax[1,0], all_times, error_stats["relative"]["attitude"])

        ax[0,1].plot(all_times, pos_err_abs, 'm-', linewidth=error_line_width)
        plotStatLines(ax[0,1], all_times, error_stats["absolute"]["position"])
        ax[1,1].plot(all_times, att_err_abs, 'm-', linewidth=error_line_width)
        plotStatLines(ax[1,1], all_times, error_stats["absolute"]["position"])

        ax[0,0].set_title("Relative Position Error (m)")
        ax[1,0].set_title("Relative Attitude Error (deg)")
        ax[0,1].set_title("Absolute Position Error (m)")
        ax[1,1].set_title("Absolute Attitude Error (deg)")
        for a in ax.ravel():
            a.set_xlim([all_times[0], all_times[-1]])
            a.set_ylim([0, None])

        # Plot the relative translation and rotation
        fig, ax = plt.subplots(3,2)
        ax[0,0].plot(all_times[:-1], tru_vel[3,:], 'r',
                    all_times[:-1], est_vel[3,:], 'r--')
        ax[1,0].plot(all_times[:-1], tru_vel[4,:], 'g',
                    all_times[:-1], est_vel[4,:], 'g--')
        ax[2,0].plot(all_times[:-1], tru_vel[5,:], 'b',
                    all_times[:-1], est_vel[5,:], 'b--')
        ax[0,1].plot(all_times[:-1], tru_vel[0,:], 'r',
                    all_times[:-1], est_vel[0,:], 'r--')
        ax[1,1].plot(all_times[:-1], tru_vel[1,:], 'g',
                    all_times[:-1], est_vel[1,:], 'g--')
        ax[2,1].plot(all_times[:-1], tru_vel[2,:], 'b',
                    all_times[:-1], est_vel[2,:], 'b--')

        ax[0,0].set_ylabel("V_x (m/s)")
        ax[1,0].set_ylabel("V_y (m/s)")
        ax[2,0].set_ylabel("V_z (m/s)")
        ax[0,0].set_title("Position Change")
        ax[2,0].set_xlabel('Time')
        ax[0,1].set_ylabel("Omega_x (deg/s)")
        ax[1,1].set_ylabel("Omega_y (deg/s)")
        ax[2,1].set_ylabel("Omega_z (deg/s)")
        ax[0,1].set_title("Attitude Change")
        ax[2,1].set_xlabel('Time')
        for a in ax.ravel():
            a.set_xlim((all_times[0], all_times[-2]))

        # Plot the absolute translation and rotation
        fig,ax = plt.subplots(3,2)
        est_pos = np.hstack([P.x().as_vector() for P in est_traj.get_elements()])
        tru_pos = np.hstack([P.x().as_vector() for P in tru_traj.get_elements()])
        ax[0,0].plot(all_times, tru_pos[0,:], 'r',
                    all_times, est_pos[0,:], 'r--')
        ax[1,0].plot(all_times, tru_pos[1,:], 'g',
                    all_times, est_pos[1,:], 'g--')
        ax[2,0].plot(all_times, tru_pos[2,:], 'b',
                    all_times, est_pos[2,:], 'b--')
        est_att = np.hstack([np.reshape(P.R().as_euler(), (3,1)) for P in est_traj.get_elements()])
        tru_att = np.hstack([np.reshape(P.R().as_euler(), (3,1)) for P in tru_traj.get_elements()])
        ax[0,1].plot(all_times, tru_att[0,:], 'r',
                    all_times, est_att[0,:], 'r--')
        ax[1,1].plot(all_times, tru_att[1,:], 'g',
                    all_times, est_att[1,:], 'g--')
        ax[2,1].plot(all_times, tru_att[2,:], 'b',
                    all_times, est_att[2,:], 'b--')

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
            a.set_xlim((all_times[0], all_times[-1]))
        for i in range(3):
            ax[i,1].set_ylim((-np.pi, np.pi))

        # Plot the full trajectories (position 3d)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(tru_pos[0,:], tru_pos[1,:], tru_pos[2,:], 'r')
        ax.plot(est_pos[0,:], est_pos[1,:], est_pos[2,:], 'b--')

        ax.set_title("Estimated vs. True Trajectory")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        ax.legend(["True", "Est."])
        ax.set_box_aspect(np.max(est_pos, axis=1)-np.min(est_pos,axis=1) + np.ones(3)*2)

        # Plot the full trajectories (position xy)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(tru_pos[0,:], tru_pos[1,:], 'r')
        ax.plot(est_pos[0,:], est_pos[1,:], 'b--')

        ax.set_title("Estimated vs. True Trajectory")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(["True", "Est."])
        ax.set_aspect("equal")

        plt.show()
