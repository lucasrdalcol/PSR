#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import cv2


# --------------------------------------------------
# A simple python script to load and read an image using OpenCV
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------
import numpy as np
from numpy import uint8


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

    ranges = {'b': {'min': 20, 'max': 150},
              'g': {'min': 20, 'max': 150},
              'r': {'min': 20, 'max': 150}}

    mins = np.array([ranges['b']['min'], ranges['g']['min'], ranges['r']['min']])
    maxs = np.array([ranges['b']['max'], ranges['g']['max'], ranges['r']['max']])
    image_processed = cv2.inRange(image_rgb, mins, maxs)

    cv2.namedWindow('Detect green box', cv2.WINDOW_NORMAL)
    cv2.imshow('Detect green box', image_processed)  # Display the image

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
