#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import cv2
import numpy as np
from functools import partial


# --------------------------------------------------
# A simple python script to load and read an image using OpenCV
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# Global variables
min_BH = 0
max_BH = 0
min_GS = 0
max_GS = 0
min_RV = 0
max_RV = 0


def onTrackbar(threshold, image_gray, window_name):
    """
    Binarize image using a trackbar callback function
    :param threshold: the threshold of the binarization. data type: int
    """

    # Binarize image and show using the given threshold at the time
    _, image_thresholded = cv2.threshold(image_gray, threshold, 255, cv2.THRESH_BINARY)
    cv2.imshow(window_name, image_thresholded)  # Display the image again


def minOnTrackbarBH(var):
    global min_BH
    min_BH = var
    print(min_BH)
    return min_BH


def maxOnTrackbarBH(var):
    global max_BH
    max_BH = var
    return max_BH


def minOnTrackbarGS(var):
    return min_GS


def maxOnTrackbarGS(var):
    return max_GS


def minOnTrackbarRV(var):
    return min_RV


def maxOnTrackbarRV(var):
    return max_RV


# function which will be called on mouse input
def onMouse(action, x, y, flags, param):
    """
    Function called by cv2.setMouseCallback to print the coordinates of where you clicked in the image
    :param action: to click a button in the mouse, specifically the left button down
    :param x: the x coordinate of the image where you clicked
    :param y: the y coordinate of the image where you clicked
    :param flags:
    :param param:
    """
    if action == cv2.EVENT_LBUTTONDOWN:
        print('The coordinates of where you clicked are: x = ' + str(x) + ' , y = ' + str(y))


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, help="Input full path image filename")
    ap.add_argument('-hsv', '--hsv', action='store_true', help="Use HSV image or not. If not is RGB image")
    args = vars(ap.parse_args())

    # Load image with given filename and show
    image_filename = args['filename']
    image_rgb = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image_rgb)  # Display the image

    # Convert image to HSV color space if wanted
    image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2HSV)

    # Create window first
    window_name = 'Segmented'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Create a trackbar to control the threshold of the binarization
    cv2.createTrackbar('min B/H', window_name, 0, 255, minOnTrackbarBH)
    cv2.createTrackbar('max B/H', window_name, 0, 255, maxOnTrackbarBH)
    cv2.createTrackbar('min G/S', window_name, 0, 255, minOnTrackbarGS)
    cv2.createTrackbar('max G/S', window_name, 0, 255, maxOnTrackbarGS)
    cv2.createTrackbar('min R/V', window_name, 0, 255, minOnTrackbarRV)
    cv2.createTrackbar('max R/V', window_name, 0, 255, maxOnTrackbarRV)

    minOnTrackbarBH(0)
    maxOnTrackbarBH(255)
    minOnTrackbarGS(0)
    maxOnTrackbarGS(255)
    minOnTrackbarRV(0)
    maxOnTrackbarRV(255)

    while True:
        # Establish ranges for each channel to create a mask
        ranges = {'b': {'min': min_BH, 'max': max_BH},
                  'g': {'min': min_GS, 'max': max_GS},
                  'r': {'min': min_RV, 'max': max_RV}}

        # Convert the dict structure created before to numpy arrays, because opencv uses it.
        mins = np.array([ranges['b']['min'], ranges['g']['min'], ranges['r']['min']])
        maxs = np.array([ranges['b']['max'], ranges['g']['max'], ranges['r']['max']])

        # Create mask using cv2.inRange. The output is still in uint8
        segmented = cv2.inRange(image_rgb, mins, maxs)

        # Show segmented image
        cv2.imshow(window_name, segmented)  # Display the image

        # Get the coordinates of where you clicked in the image
        cv2.setMouseCallback(window_name, onMouse)

        cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
