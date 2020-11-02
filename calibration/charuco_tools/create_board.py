#!/usr/bin/env python3

import cv2

def default_dictionary():
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

def default_board(square_size, aruco_dict=cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)):
    squares_x = 7
    squares_y = 5
    return cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_size, square_size * 0.8, aruco_dict)



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Create a Charuco Board")
    parser.add_argument("--size", default="A4", type=str, help="Size of the charuco board printout.")
    parser.add_argument("--bits", default=4, type=int, help="Which aruco marker size to use")
    args = parser.parse_args()

    if args.bits == 4:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    elif args.bits == 5:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    elif args.bits == 6:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    else:
        raise ValueError("Aruco bits must be 4, 5 or 6.")

    margin_size = 0.025
    squares_x = 7
    squares_y = 5
    if args.size == "A4":
        square_size = min((0.594-margin_size*2) /squares_x, (0.420-margin_size*2)/squares_y)
    else:
        raise ValueError("Paper size must be A4.")

    board = default_board(square_size, aruco_dict)

    img = board.draw((200*squares_x, 200*squares_y))

    cv2.imwrite('charuco.png',img)


