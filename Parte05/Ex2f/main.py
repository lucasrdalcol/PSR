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

    # Establish ranges for each channel to create a mask
    ranges = {'b': {'min': 20, 'max': 60},
              'g': {'min': 80, 'max': 140},
              'r': {'min': 0, 'max': 45}}

    # Convert the dict structure created before to numpy arrays, because opencv uses it.
    mins = np.array([ranges['b']['min'], ranges['g']['min'], ranges['r']['min']])
    maxs = np.array([ranges['b']['max'], ranges['g']['max'], ranges['r']['max']])

    # Create mask using cv2.inRange. The output is still in uint8
    mask = cv2.inRange(image_rgb, mins, maxs)

    # Convert using numpy from uint8 to bool
    mask = mask.astype(bool)

    # Process a copy of the image using the mask created.
    image_processed = copy(image_rgb)
    image_processed[mask] = image_processed[mask] + 245  # Be careful with under 0 and over 255 when summing.

    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask', 255 * mask.astype(np.uint8))  # Display the image

    cv2.namedWindow('Image Processed', cv2.WINDOW_NORMAL)
    cv2.imshow('Image Processed', image_processed)  # Display the image

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
