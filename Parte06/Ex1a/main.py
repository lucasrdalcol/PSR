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
#
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

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, help="Input full path image filename")
    args = vars(ap.parse_args())

    # Load image with given filename and show
    image_filename = args['filename']
    image_rgb = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image_rgb)  # Display the image

    # Draw a circle in the middle of the image
    height = image_rgb.shape[0]
    width = image_rgb.shape[1]
    cv2.circle(img=image_rgb, center=(round(width/2), round(height/2)), radius=50, color=(0, 0, 255), thickness=1)

    cv2.namedWindow('Original with a circle', cv2.WINDOW_NORMAL)
    cv2.imshow('Original with a circle', image_rgb)  # Display the image again

    cv2.waitKey(0)


if __name__ == "__main__":
    main()
