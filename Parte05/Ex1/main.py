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

def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, action='append', help="Input cloud points .e57 file")
    args = vars(ap.parse_args())

    # Exercise 1a and 1b
    image_filename = args['filename'][0]
    image = cv2.imread(image_filename, cv2.IMREAD_COLOR)  # Load an image

    cv2.imshow('window', image)  # Display the image
    cv2.waitKey(0)  # wait a key

    # Exercise 1c

    # # Option A: showing only once each image
    # for image_filename in args['filename']:
    #     image = cv2.imread(image_filename, cv2.IMREAD_COLOR)  # Load an image
    #
    #     cv2.imshow('window', image)  # Display the image
    #     cv2.waitKey(3000)  # wait 3 seconds before continue or wait a key

    # # Option B: showing all the time alternating between the two images
    # print("press 'q' to quit.")
    # global pressed_key
    # while True:
    #     for image_filename in args['filename']:
    #         image = cv2.imread(image_filename, cv2.IMREAD_COLOR)  # Load an image
    #
    #         cv2.imshow('window', image)  # Display the image
    #         pressed_key = cv2.waitKey(3000)  # wait 3 seconds before continue or wait a key
    #
    #     if pressed_key == ord('q'):
    #         break

if __name__ == "__main__":
    main()
