import cv2

calibFile = cv2.FileStorage("calib_test2", cv2.FILE_STORAGE_READ)

K = calibFile.getNode("camera_matrix")
K = K.mat()

dist = calibFile.getNode("distortion_coefficients")
dist = dist.mat()

print(K)
print(dist)

cap = cv2.VideoCapture("charuco2.mp4")

flag, frame = cap.read()
while(flag):
    goodFrame = cv2.undistort(frame, K, dist)
    cv2.imshow("undistorted", goodFrame)
    cv2.waitKey(1)

    flag, frame = cap.read()