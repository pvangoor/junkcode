#!/usr/bin/env python3
import argparse
import csv
from pylie import SE3
from scipy.spatial.transform import Rotation
import numpy as np

parser = argparse.ArgumentParser("Convert an ardupilot VIO dataset into a rosbag.")
parser.add_argument("mav", metavar='m', help="The file containing the MAV data.")
parser.add_argument("--enu", action='store_true', help="Use ENU instead of NED.")
args = parser.parse_args()

def ned2enu(position):
    n = position[0]
    e = position[1]
    u = - position[2]
    return [e, n, u]

def lat2radius(lat):
    equatorial_radius = 6378.1370e3 # metres
    polar_radius = 6356.7523e3 # metres

    c = np.cos(lat * np.pi / 180.0)
    s = np.sin(lat * np.pi / 180.0)

    radius = (equatorial_radius**2 * c)**2 + (polar_radius**2 * s)**2
    radius = radius / ((equatorial_radius * c)**2 + (polar_radius * s)**2)
    radius = np.sqrt(radius)
    return float(radius)

def gps2xyz(gps_origin, gps):
    # The origin defines the frame NED using a local approximation
    
    local_radius = lat2radius(gps_origin[0])

    nor = float(np.sin((gps[0]- gps_origin[0]) * np.pi / 180.0) * local_radius)
    eas = float(np.sin((gps[1]- gps_origin[1]) * np.pi / 180.0) * local_radius)
    down = gps_origin[2] - gps[2]

    return [nor, eas, down]
    


print("Reading GPS and attitude data.")
with open(args.mav, 'r') as f:
    reader = csv.reader(f)
    next(reader) # Skip header

    times = []
    att_data = []
    gps_data = []

    for row in reader:
        t = float(row[0])
        att = [float(row[i]) for i in range(8,11)]
        gps = [float(row[i]) for i in range(11,14)]

        # Avoid repeat entries
        if len(times) > 0 and all(att_data[-1][i] == att[i] for i in range(3)) and all(gps_data[-1][i] == gps[i] for i in range(3)):
            continue

        times.append(t)
        att_data.append(att)
        gps_data.append(gps)

# Convert into wx pose format and write
output_fname = "mav_poses.csv"
with open(output_fname, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time (s)", "tx", "ty", "tz", "qw", "qx", "qy", "qz"])
    N = len(times)
    for i in range(N):
        position = gps2xyz(gps_data[0], gps_data[i])
        if args.enu:
            position = ned2enu(position)

        attitude = Rotation.from_euler('xyz', att_data[i])
        pose = SE3(attitude, position)
        writer.writerow([times[i]] + pose.to_list('xw'))

    

