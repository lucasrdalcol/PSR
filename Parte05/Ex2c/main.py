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

    b, g, r = cv2.split(image_rgb)
    _, b_thresholded = cv2.threshold(b, 50, 255, cv2.THRESH_BINARY)
    _, g_thresholded = cv2.threshold(g, 100, 255, cv2.THRESH_BINARY)
    _, r_thresholded = cv2.threshold(r, 150, 255, cv2.THRESH_BINARY)

    image_thresholded = cv2.merge((b_thresholded, g_thresholded, r_thresholded))
    cv2.namedWindow('After binarize each channel', cv2.WINDOW_NORMAL)
    cv2.imshow('After binarize each channel', image_thresholded)  # Display the image

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
