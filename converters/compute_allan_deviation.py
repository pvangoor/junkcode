#!/usr/bin/env python3
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
import progressbar

parser = argparse.ArgumentParser(
    "Compute the Allan Deviation of an IMU sensor")
parser.add_argument("imu", metavar='i',
                    help="The file contained stamps IMU data.")
parser.add_argument("--resolution", default=50,
                    help="The number of bin lengths to use.")
args = parser.parse_args()


def allan_variance(dt, stamps_list, data):
    total_time = stamps_list[-1] - stamps_list[0]
    bins = int(total_time / dt)


    # Collect bins
    bin_datas = []
    b = -1
    for (i, stamp) in enumerate(stamps_list):
        if stamp >= dt * (b + 1) + stamps[0]:
            # Create new bin!
            bin_datas.append([])
            b += 1
        
        bin_datas[-1].append(data[i:i+1,:])
    bin_datas = [np.vstack(bd) for bd in bin_datas]
    

    # bin_datas = [data[((stamps >= stamps[0, 0]+dt*i) & (stamps <
                                                        # stamps[0, 0]+dt*(i+1))).ravel(), :] for i in range(bins)]
    data_means = np.vstack([np.mean(bin_data, axis=0)
                            for bin_data in bin_datas if len(bin_data) > 0])

    AVAR = 1 / (2 * (len(data_means) - 1)) * \
        np.sum(np.diff(data_means, axis=0)**2, axis=0)

    return AVAR


print("Importing data")
with open(args.imu, 'r') as f:
    reader = csv.reader(f)
    next(reader)

    stamps = []
    gyr = []
    acc = []
    for row in reader:
        stamps.append(float(row[0]))
        gyr.append(np.array([float(row[i]) for i in range(1, 4)]))
        acc.append(np.array([float(row[i]) for i in range(4, 7)]))

    stamps = np.reshape(stamps, (-1, 1))
    gyr = np.vstack(gyr)
    acc = np.vstack(acc)

# Compute the Allan variance
print("Computing Allan deviation...")
tau_min = np.mean(np.diff(stamps, axis=0))
tau_max = (stamps[-1] - stamps[0]) / 2.1

# tau = np.linspace(tau_min, tau_max, args.resolution).ravel().tolist()
tau = np.logspace(np.log10(tau_min), np.log10(tau_max), args.resolution).ravel().tolist()
gyr_deviations = []
acc_deviations = []
for dt in progressbar.progressbar(tau):
    gyr_deviations.append(np.sqrt(allan_variance(dt, stamps, gyr)))
    acc_deviations.append(np.sqrt(allan_variance(dt, stamps, acc)))
gyr_deviations = np.vstack(gyr_deviations)
acc_deviations = np.vstack(acc_deviations)

print(
"""Instructions

The random walk (white noise) is found by fitting a straight line to the first segment of the deviation curve, and taking its value at t=1s. This program reports directly the value of the curve at 1s, which may not be exactly right. Check!
The bias instability is found as the minimum value of the curve.
"""
)

one_second_idx = np.argmin(abs(np.array(tau)-1))
print("Gyroscope Random Walk")
print("x-axis: {:.3e} rad/sqrt(s)".format(float(gyr_deviations[one_second_idx,0])))
print("y-axis: {:.3e} rad/sqrt(s)".format(float(gyr_deviations[one_second_idx,1])))
print("z-axis: {:.3e} rad/sqrt(s)".format(float(gyr_deviations[one_second_idx,2])))
print("Gyroscope Bias Instability")
print("x-axis: {:.3e} rad/s (at {:.2f} s)".format(float(np.min(gyr_deviations[:,0])), tau[np.argmin(gyr_deviations[:,0])]))
print("y-axis: {:.3e} rad/s (at {:.2f} s)".format(float(np.min(gyr_deviations[:,1])), tau[np.argmin(gyr_deviations[:,1])]))
print("z-axis: {:.3e} rad/s (at {:.2f} s)".format(float(np.min(gyr_deviations[:,2])), tau[np.argmin(gyr_deviations[:,2])]))
print()
print("Accelerometer Random Walk")
print("x-axis: {:.3e} m/s/sqrt(s)".format(float(acc_deviations[one_second_idx,0])))
print("y-axis: {:.3e} m/s/sqrt(s)".format(float(acc_deviations[one_second_idx,1])))
print("z-axis: {:.3e} m/s/sqrt(s)".format(float(acc_deviations[one_second_idx,2])))
print("Accelerometer Bias Instability")
print("x-axis: {:.3e} m/s/s (at {:.2f} s)".format(float(np.min(acc_deviations[:,0])), tau[np.argmin(acc_deviations[:,0])]))
print("y-axis: {:.3e} m/s/s (at {:.2f} s)".format(float(np.min(acc_deviations[:,1])), tau[np.argmin(acc_deviations[:,1])]))
print("z-axis: {:.3e} m/s/s (at {:.2f} s)".format(float(np.min(acc_deviations[:,2])), tau[np.argmin(acc_deviations[:,2])]))


fig, ax = plt.subplots(2,1)
ax[0].plot(tau, gyr_deviations)
ax[0].set_xscale('log')
ax[0].set_yscale('log')
ax[0].set_title("Gyroscope Allan Deviation")
ax[0].set_xlabel("Bin Time (s)")
ax[0].set_ylabel("Allan deviation")
ax[0].legend(["x", "y", "z"])
ax[0].set_xlim([tau_min, None])
ax[0].grid(True, which='both')

ax[1].plot(tau, acc_deviations)
ax[1].set_xscale('log')
ax[1].set_yscale('log')
ax[1].set_title("Accelerometer Allan Deviation")
ax[1].set_xlabel("Bin Time (s)")
ax[1].set_ylabel("Allan deviation")
ax[1].legend(["x", "y", "z"])
ax[1].set_xlim([tau_min, None])
ax[1].grid(True, which='both')

plt.tight_layout()
plt.show()
