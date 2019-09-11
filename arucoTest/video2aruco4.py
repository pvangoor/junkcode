import cv2
import csv
import argparse
import progressbar

parser = argparse.ArgumentParser(description="Extract ARUCO markers at every frame of a video file")
parser.add_argument("videoName", metavar="v", type=str, nargs=1, help="The name of the video file to be processed.")
parser.add_argument("camera", metavar="c", type=str, nargs=1, help="The file containing camera calibration.")
parser.add_argument("markerLength", metavar="m", type=float, nargs=1, help="The side length of the aruco markers.")
args = parser.parse_args()

video_fname = args.videoName[0]
cap = cv2.VideoCapture(video_fname)
frame_rate = cap.get(cv2.CAP_PROP_FPS)
total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

calibFile = cv2.FileStorage(args.camera[0], cv2.FILE_STORAGE_READ)
K = calibFile.getNode("camera_matrix")
K = K.mat()
dist = calibFile.getNode("distortion_coefficients")
dist = dist.mat()

marker_length = args.markerLength[0]


outputFname = video_fname[:-4] + ".csv"
outputFile = open(outputFname, "w")
outputWriter = csv.writer(outputFile)
header_row = ["frame", "time", "id", "x", "y", "z", "rx", "ry", "rz"]
outputWriter.writerow(header_row)

video_count = 0
video_time = 0

progBar = progressbar.ProgressBar(max_value=total_frames)

flag, frame = cap.read()
while flag:
    [corners, ids, rejectedPts] = cv2.aruco.detectMarkers(frame, aruco_dict)

    if ids is not None:
        [rvecs, tvecs] = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, K, dist)
        
        marker_number = ids.shape[0]
        for i in range(marker_number):
            marker_id = ids[i,0]
            marker_pos = [tvecs[i,0,j] for j in range(3)]
            marker_rot = [rvecs[i,0,j] for j in range(3)]

            row = [video_count, video_time, marker_id] + marker_pos + marker_rot
            outputWriter.writerow(row)

    flag, frame = cap.read()
    video_time += 1/frame_rate
    video_count += 1
    progBar.update(video_count)

outputFile.close()