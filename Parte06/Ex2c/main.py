#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import json
import cv2
import numpy as np
from functools import partial


# --------------------------------------------------
# Script to capture and save webcam video
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# ---------------------------------------------------
# Global Variables
# ---------------------------------------------------


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    # initial setup
    capture = cv2.VideoCapture(0)  # Selecting the camera 0
    window_name = 'Webcam'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    print('Start capturing the webcam video.')
    print('press q to exit.')
    while capture.isOpened():
        ret, frame = capture.read()  # get an image from the camera (a frame)

        # Check if the capture read is giving return
        if ret is True:
            # write the frame
            out.write(frame)

            # Show image
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1)

            # key to break the cycle
            if key == ord('q'):
                print('You pressed q, exiting the program')
                break

        else:
            break

    # Release everything if job is finished
    capture.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
