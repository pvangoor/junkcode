#!/usr/bin/env python3

# Calibrate a camera from a video with a charuco board

import argparse
import cv2
from charuco_create_board import default_board, default_dictionary

parser = argparse.ArgumentParser(description="Calibrate a camera from a charuco board video")
parser.add_argument('video', type=str, help="The video file name.")
parser.add_argument('--size', type=float, default=39.3, help="The size of the squares on the board in mm.")
parser.add_argument('--max_frames', type=int, default=50, help="The maximum number of frames to use when calibrating. Default 50.")
args = parser.parse_args()

aruco_dict = default_dictionary()
board = default_board(args.size)

print("Detecting charuco board in the video.")
cap = cv2.VideoCapture(args.video)

all_corners = []
all_ids = []

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
        continue

    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if retval < 8:
        print("Not enough charuco corners detected in frame {}.".format(frame_counter))
        continue

    all_corners.append(charuco_corners)
    all_ids.append(charuco_ids)

# Calibrate the camera
# Limit the number of frames used
if len(all_corners) > args.max_frames:
    step = len(all_corners) // (args.max_frames + 1)
    all_corners = all_corners[::step]
    all_ids = all_ids[::step]

print("Calibrating the camera from {} frames.".format(len(all_corners)))
retval, camera_matrix, dist_coeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors = cv2.aruco.calibrateCameraCharucoExtended(all_corners, all_ids, board, image_size, None, None)

print("Camera Matrix")
print(camera_matrix)
print("Distortion Parameters")
print(dist_coeffs)

try:
    import yaml
except ImportError:
    print("Could not import yaml.")
    exit(0)

data_dict = {
    "camera_matrix":camera_matrix.tolist(),
    "dist_coeffs":dist_coeffs.tolist(),
    "stdDeviationsIntrinsics":stdDeviationsIntrinsics.tolist(),
    "perViewErrors":perViewErrors.tolist(),
    # "stdDeviationsExtrinsics":stdDeviationsExtrinsics.tolist(),
}
ofname = args.video[:args.video.rfind('.')] + "_calibration.yaml"
with open(ofname, 'w') as f:
    yaml.safe_dump(data_dict, f)
