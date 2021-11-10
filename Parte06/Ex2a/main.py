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
# Script to capture a photo from the webcam and show
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

    _, frame = capture.read()  # get an image from the camera (a frame)

    # Show image
    cv2.imshow(window_name, frame)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
