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
    image = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image)  # Display the image

    # Convert colored image to grayscale and show
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow('Grayscale', cv2.WINDOW_NORMAL)
    cv2.imshow('Grayscale', image_gray)  # Display the image

    # Binarize image and show
    # # Exercise 2a): binarize using cv2.thresholded
    # retval, image_thresholded = cv2.threshold(image_gray, 128, 255, cv2.THRESH_BINARY)

    # Exercise 2b): binarize using comparison
    image_thresholded = image_gray > 128
    image_thresholded = image_thresholded.astype(uint8)

    cv2.namedWindow('Binarized', cv2.WINDOW_NORMAL)
    cv2.imshow('Binarized', image_thresholded)  # Display the image

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
