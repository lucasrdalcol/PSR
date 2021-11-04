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

    ranges = {'b': {'min': 0, 'max': 100},
              'g': {'min': 80, 'max': 256},
              'r': {'min': 0, 'max': 100}}

    mins = np.array([ranges['b']['min'], ranges['g']['min'], ranges['r']['min']])
    maxs = np.array([ranges['b']['max'], ranges['g']['max'], ranges['r']['max']])
    mask = cv2.inRange(image_rgb, mins, maxs)

    # Convert using numpy using uint8 to bool
    mask = mask.astype(np.bool)

    image_processed = copy(image_rgb)
    image_processed[~mask] = (image_processed[~mask] * 0.2).astype(np.uint8)

    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask', 255 * mask.astype(np.uint8))  # Display the image

    cv2.namedWindow('Image Processed', cv2.WINDOW_NORMAL)
    cv2.imshow('Image Processed', image_processed)  # Display the image

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
