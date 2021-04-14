#!/usr/bin/env python3
import cv2
import os
import argparse
import progressbar

# Read distortion parameters


def read_camera_parameters(fname: str):
    try:
        fs = cv2.FileStorage(fname, cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
    except SystemError:
        import yaml
        import numpy as np
        with open(fname, 'r') as f:
            data_dict = yaml.safe_load(f)
        camera_matrix = np.reshape(data_dict["camera_matrix"], (3, 3))
        dist_coeffs = np.array(data_dict["dist_coeffs"]).ravel()

    return camera_matrix, dist_coeffs


def create_undistortion_maps(img, full_size=False):
    img_size = (img.shape[1], img.shape[0])
    udist_size = img_size
    if full_size:
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, img_size, 1.0)
        remaps = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_camera_matrix, img_size, 5
        )
        img_undist = cv2.remap(img, remaps[0], remaps[1], cv2.INTER_LINEAR)
        udist_size = (img_undist.shape[1], img_undist.shape[0])
    else:
        remaps = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, camera_matrix, img_size, 5
        )
    return remaps, udist_size


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Undistort given image(s).")
    parser.add_argument('parameters', metavar='p',
                        type=str, help="The camera parameters.")
    parser.add_argument('file_name', metavar='f', nargs='+',
                        type=str, help="The image(s) to undistort.")
    parser.add_argument('--video', action='store_true',
                        help="Set this flag to convert videos instead.")
    parser.add_argument('--full_size', action='store_true',
                        help="Set this flag to show the full undistorted frame.")
    args = parser.parse_args()

    # Import camera parameters
    camera_matrix, dist_coeffs = read_camera_parameters(args.parameters)
    print("Using Camera Matrix\n", camera_matrix)
    print("Using Distortion Coefficients\n", dist_coeffs)

    # Prepare undistortion map using the first image.
    remaps = None

    for fname in args.file_name:
        ext_pos = fname.rfind('.')
        new_fname = fname[:ext_pos] + "_undistorted" + fname[ext_pos:]

        if args.video:
            # Handle videos
            cap = cv2.VideoCapture(fname)
            ret, img = cap.read()
            remaps, udist_size = create_undistortion_maps(img, args.full_size)

            writer = cv2.VideoWriter(new_fname, cv2.VideoWriter_fourcc(
                *"X264"), cap.get(cv2.CAP_PROP_FPS), udist_size, isColor=(len(img.shape)>2))

            while(ret):
                img_undist = cv2.remap(img, remaps[0], remaps[1], cv2.INTER_LINEAR)
                writer.write(img_undist)

                ret, img = cap.read()
            
            print("Closing...")
            cap.release()
            writer.release()

        else:
            # Handle images
            img_undist = cv2.remap(img, remaps[0], remaps[1], cv2.INTER_LINEAR)
            cv2.imwrite(new_fname, img_undist)

