#!/usr/bin/env python3
import cv2
import os
import argparse
import progressbar

parser = argparse.ArgumentParser(description="Undistort given image(s).")
parser.add_argument('f',  type=str, help="The image(s) to undistort.")
parser.add_argument('p',  type=str, help="The camera parameters.")
parser.add_argument('--folder', action='store_true', help="Used to indicate we are converting a folder. Default: False.")
parser.add_argument('--video', action='store_true', help="Used to indicate we are converting a video. Default: False.")
parser.add_argument('--original_size', action='store_true', help="Used to indicate we want the frame to remain the same size. Default: False.")
parser.add_argument('--extension', metavar='e',  type=str, default="png", help="The extension of the files to convert. Default: png.")
args = parser.parse_args()

# Read distortion parameters
try:
    fs = cv2.FileStorage(args.p, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("dist_coeffs").mat()
except SystemError:
    import yaml
    import numpy as np
    with open(args.p, 'r') as f:
        data_dict = yaml.safe_load(f)
    camera_matrix = np.reshape(data_dict["camera_matrix"], (3,3))
    dist_coeffs = np.array(data_dict["dist_coeffs"]).ravel()

print("Using Camera Matrix\n", camera_matrix)
print("Using Distortion Coefficients\n", dist_coeffs)


fname = args.f
folder_flag = args.folder
# Check if file or folder
if not folder_flag and not args.video:
    img = cv2.imread(fname)
    img_undist = cv2.undistort(img, camera_matrix, dist_coeffs)
    
    ext_pos = fname.rfind('.')
    out_str = fname[:ext_pos] + "_undistorted" + fname[ext_pos:]
    cv2.imwrite(out_str, img_undist)
    exit(0)

# Deal with videos
if args.video:
    cap = cv2.VideoCapture(fname)
    ext_pos = fname.rfind('.')
    new_fname = fname[:ext_pos] + "_undistorted" + fname[ext_pos:]

    ret, img = cap.read()

    img_size = (img.shape[1], img.shape[0])
    if not args.original_size:
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, img_size, 1.0)
        remaps = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_camera_matrix, img_size, 5
        )
        # img_undist = cv2.undistort(img, camera_matrix, dist_coeffs, new_camera_matrix)
        img_undist = cv2.remap(img, remaps[0], remaps[1], cv2.INTER_LINEAR)
        img_size = (img_undist.shape[1], img_undist.shape[0])

    writer = cv2.VideoWriter(new_fname, cv2.VideoWriter_fourcc('M','J','P','G'), cap.get(cv2.CAP_PROP_FPS), img_size, False)

    while ret:
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if args.original_size:
            img_undist = cv2.undistort(img, camera_matrix, dist_coeffs)
        else:
            img_undist = cv2.remap(img, remaps[0], remaps[1], cv2.INTER_LINEAR)
        writer.write(img_undist)
        ret, img = cap.read()
    
    cap.release()
    writer.release()
    exit(0)
    

# If the program reaches here, we are dealing with a folder
new_folder = fname[:-1]+"_undistorted/"
if not os.path.exists(new_folder):
    os.mkdir(new_folder)

ext_str = args.extension
file_names = os.listdir(fname)
file_names = [f for f in file_names if f.endswith(ext_str)]
for img_name in progressbar.progressbar(file_names):
    img = cv2.imread(os.path.join(fname, img_name))
    img_undist = cv2.undistort(img, camera_matrix, dist_coeffs)
    cv2.imwrite(os.path.join(new_folder, img_name), img_undist)


