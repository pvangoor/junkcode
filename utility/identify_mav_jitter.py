#!/usr/bin/env python3
import argparse
import csv

import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser("Identify jitter in a mav file.")
parser.add_argument("mav", metavar='m', help="The file containing the MAV data.")
args = parser.parse_args()

print("Reading MAV time stamps.")
with open(args.mav, 'r') as f:
    reader = csv.reader(f)
    next(reader) # Skip header

    times_cpu = []
    times_mav = []

    row0 = next(reader)
    cpu_t0 = float(row0[0])
    mav_t0 = float(row0[7])

    for row in reader:

        times_cpu.append(float(row[0]) - cpu_t0)
        times_mav.append(float(row[7]) - mav_t0)

time_diffs = [t_cpu - t_mav for (t_cpu, t_mav) in zip(times_cpu, times_mav)]
cpu_steps = np.diff(times_cpu)
mav_steps = np.diff(times_mav)

fig, ax = plt.subplots(4,1)

ax[0].hist(time_diffs, 1000)
ax[0].set_title("Differences between relative times.")

ax[1].hist(cpu_steps, 1000)
ax[1].set_title("CPU time steps")

ax[2].hist(mav_steps, 1000)
ax[2].set_title("MAV time steps")

ax[3].plot(times_cpu, time_diffs)
ax[3].set_title("Relative time differences over cpu time")


plt.show()