#!/usr/bin/env python3

# Calibrate a camera from a video with a charuco board

import argparse
import csv
import cv2
import numpy as np
import yaml
from pylie import SE3
from charuco_create_board import default_board, default_dictionary

parser = argparse.ArgumentParser(description="Compute the poses of the charuco board with respect to the camera.")
parser.add_argument('video', type=str, help="The video file name.")
parser.add_argument('stamps', type=str, help="The timestamps file name.")
parser.add_argument('intrinsics', type=str, help="The camera intrinsics file name")
parser.add_argument('--size', type=float, default=39.3, help="The size of the squares on the board in mm.")
parser.add_argument('--cam_poses', action='store_true', help="Set this flag to record camera poses w.r.t. the board instead.")
args = parser.parse_args()

aruco_dict = default_dictionary()
board = default_board(args.size)

# Read intrinsics
print("Reading intrinsics.")
with open(args.intrinsics, 'r') as f:
    intrinsics_dict = yaml.safe_load(f)
    camera_matrix = np.array(intrinsics_dict["camera_matrix"])
    dist_coeffs = np.array(intrinsics_dict["dist_coeffs"])

# Read stamps
print("Reading time stamps.")
with open(args.stamps, 'r') as f:
    next(f) # Skip header
    stamps = [int(row) for row in f]

# Read charuco poses
print("Detecting charuco poses.")
cap = cv2.VideoCapture(args.video)
rvecs = []
tvecs = []
frame_counter = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame_counter += 1

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict)

    if len(corners) == 0:
        print("No corners detected in frame {}.".format(frame_counter))
        rvecs.append(None)
        tvecs.append(None)
        continue

    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if retval < 4:
        print("Not enough charuco corners detected in frame {}.".format(frame_counter))
        rvecs.append(None)
        tvecs.append(None)
        continue

    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None)
    if not retval:
        print("Could not detect pose in frame {}.".format(frame_counter))
        rvecs.append(None)
        tvecs.append(None)
        continue

    rvecs.append(rvec)
    tvecs.append(tvec * 1e-3) # Adjust for mm

# Write the poses to file
ofname = args.video[:args.video.rfind('.')] + "_poses.csv"
print("Writing poses to {}.".format(ofname))
with open(ofname, 'w') as f:
    writer = csv.writer(f)
    header = "ts [ns], tx [m], ty [m], tz [m], qw, qx, qy, qz".split(",")
    writer.writerow(header)
    for stamp, rvec, tvec in zip(stamps, rvecs, tvecs):
        if rvec is None or tvec is None:
            continue
        Rmat, _ = cv2.Rodrigues(rvec)
        pose = SE3()
        pose._R = pose._R.from_matrix(Rmat)
        pose._x = pose._x.from_list(tvec.ravel().tolist())

        if args.cam_poses:
            row = [stamp] + pose.inv().to_list('xw')
        else:
            row = [stamp] + pose.to_list('xw')
        writer.writerow(row)


