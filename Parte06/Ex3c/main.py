#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import json
import cv2
import numpy as np
from functools import partial
from imutils import face_utils
import imutils
import dlib
import face_recognition
from scipy.spatial import distance


# --------------------------------------------------
# Exercise 3 complete - https://github.com/miguelriemoliveira/psr_21-22/tree/main/Parte06
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

    # initial setup
    global dist
    capture = cv2.VideoCapture(0)  # Selecting the camera 0
    window_name = 'Webcam'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Load the cascade
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    print('Start capturing the webcam video.')
    print('press s to save the current frame.')
    print('press q to exit.')
    saved_frame_count = 0

    # Load the detector
    detector = dlib.get_frontal_face_detector()

    # Load the predictor
    predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

    # ---------------------------------------------------
    # Get video from webcam and process each frame
    # ---------------------------------------------------

    # Show the video from webcam
    while True:
        _, frame = capture.read()  # get an image from the camera (a frame)
        overlay = frame.copy()  # create a copy of the frame for transparency process

        # Convert the image to grayscale to detect the landmarks (face and mouth)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(gray, (3, 3), 0)

        # Canny Edge Detection
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)  # Canny Edge Detection

        # Detect faces in gray image
        faces_opencv = face_cascade.detectMultiScale(gray, 1.1, 5)

        # Use detector to find landmarks for the speaking in gray image
        faces_dlib = detector(gray)

        mask_edges = edges.astype(bool)  # Convert the edges from uint8 to boolean

        # Draw filled rectangle around the faces and change transparency
        for (x, y, w, h) in faces_opencv:

            # Where the rectangle should be, put False on mask
            mask_edges[y:y+h, x:x+w] = False

            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), -1)
            alpha = 0.15  # Transparency factor.
            # Following line overlays transparent rectangle over the image
            frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        # Change the pixels where we have edges to red.
        frame[mask_edges] = (0, 0, 255)  # Where the mask is true, change the pixels to red

        # Draw points of the mouth
        for face in faces_dlib:
            # Create landmark object
            landmarks = predictor(image=gray, box=face)

            # Loop through all the points
            for n in range(48, 59):
                (x1, y1) = landmarks.part(n).x, landmarks.part(n).y
                (x2, y2) = landmarks.part(n+1).x, landmarks.part(n+1).y
                # cv2.circle(img=frame, center=(x1, y1), radius=1, color=(255, 0, 0), thickness=-1)
                cv2.line(img=frame, pt1=(x1, y1), pt2=(x2, y2), color=(255, 0, 0))
            # Complete the mouth
            (x1, y1) = landmarks.part(48).x, landmarks.part(48).y
            (x2, y2) = landmarks.part(59).x, landmarks.part(59).y
            cv2.line(img=frame, pt1=(x1, y1), pt2=(x2, y2), color=(255, 0, 0))

        # Method 1: https://livecodestream.dev/post/detecting-face-features-with-python/
        # Get coordinates of the top lip and the bottom lip, and get euclidean distance between both
        for face in faces_dlib:
            landmarks = predictor(image=gray, box=face)
            (x1, y1) = landmarks.part(62).x, landmarks.part(62).y
            (x2, y2) = landmarks.part(66).x, landmarks.part(66).y
            dist = distance.euclidean([x1, y1], [x2, y2])

        # Check the distance calculated before to see if the person is speaking or not
        if dist < 6.2:
            cv2.putText(frame, 'Method 1: Silence', (40, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255))
        else:
            cv2.putText(frame, 'Method 1: You are speaking', (40, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0))

        # ---------------------------------------------------
        # Show image after all processes
        # ---------------------------------------------------

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1)

        # Save a frame pressing a key
        if key == ord('s'):
            filename = 'webcam_capture_with_face_detection' + str(saved_frame_count) + '.jpg'
            cv2.imwrite(filename, frame)
            print(filename + 'saved.')
            saved_frame_count += 1

        # key to break the cycle
        if key == ord('q'):
            print('You pressed q, exiting the program')
            break

    # Release everything if job is finished
    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
