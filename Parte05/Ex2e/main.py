#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
from copy import copy
import cv2


# --------------------------------------------------
# A simple python script to load and read an image using OpenCV
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------
import numpy as np


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, help="Input image filename")
    args = vars(ap.parse_args())

    # Load image with given filename and show
    image_filename = args['filename']
    image_rgb = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image_rgb)  # Display the image

    # Convert image to HSV color space and show
    image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2HSV)
    cv2.namedWindow('HSV', cv2.WINDOW_NORMAL)
    cv2.imshow('HSV', image_hsv)  # Display the image

    # Establish ranges for each channel to create a mask
    ranges = {'h': {'min': 60, 'max': 75},
              's': {'min': 175, 'max': 255},
              'v': {'min': 85, 'max': 135}}

    # Convert the dict structure created before to numpy arrays, because opencv uses it.
    mins = np.array([ranges['h']['min'], ranges['s']['min'], ranges['v']['min']])
    maxs = np.array([ranges['h']['max'], ranges['s']['max'], ranges['v']['max']])

    # Create mask using cv2.inRange. The output is still in uint8
    mask = cv2.inRange(image_hsv, mins, maxs)

    # Show mask image
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask', mask)  # Display the image

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
