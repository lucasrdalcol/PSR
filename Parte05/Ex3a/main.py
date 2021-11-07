#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import cv2
import numpy as np

# --------------------------------------------------
# A simple python script to load and read an image using OpenCV
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# Global variables
window_name = 'Binarizing image through a track bar'


def onTrackbar(threshold):
    """
    Binarize image using a trackbar callback function
    :param threshold: the threshold of the binarization. data type: int
    """

    # Binarize image and show using the given threshold at the time
    _, image_thresholded = cv2.threshold(image_gray, threshold, 255, cv2.THRESH_BINARY)
    cv2.imshow(window_name, image_thresholded)  # Display the image again


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, help="Input full path image filename")
    args = vars(ap.parse_args())

    # Load image with given filename and show
    image_filename = args['filename']
    image = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image)  # Display the image

    # Convert colored image to grayscale and show
    global image_gray  # Use global variable for the function onTrackbar
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow('Grayscale', cv2.WINDOW_NORMAL)
    cv2.imshow('Grayscale', image_gray)  # Display the image

    # Create window first
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Create a trackbar to control the threshold of the binarization
    trackbar_name = 'Binarize threshold'
    cv2.createTrackbar(trackbar_name, window_name, 0, 255, onTrackbar)

    # Initialize binarized image with threshold equal to 0
    onTrackbar(0)

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
