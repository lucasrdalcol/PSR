#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import json
import time
from my_functions import *
import cv2
import numpy as np
from functools import partial
from imutils import face_utils
import imutils
import dlib
import face_recognition
from scipy.spatial import distance
import pyaudio
import wave
import audioop

# --------------------------------------------------
# Exercise 3 complete - https://github.com/miguelriemoliveira/psr_21-22/tree/main/Parte06
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    capture = cv2.VideoCapture(0)  # Selecting the camera 0
    window_name = 'Webcam'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Load the cascade - OpenCV
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # Load the detector and the predictor
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

    # Counter to save how many frames we want
    saved_frame_count = 0

    # # Initialize parameters of audio
    # CHUNCK = 256
    # FORMAT = pyaudio.paInt16
    # CHANNELS = 1
    # RATE = 88200
    #
    # p = pyaudio.PyAudio()
    # stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNCK)

    print('Start capturing the webcam video and audio.')
    print('press s to save the current frame.')
    print('press q to exit.')

    # ---------------------------------------------------
    # Execution
    # ---------------------------------------------------

    # Show the video from webcam
    while True:
        # Get an image from the camera (a frame)
        _, frame = capture.read()

        # Convert the image to grayscale to detect the landmarks (face and mouth)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(gray, (3, 3), 0)

        # Detect faces, edges off the face and mouths.
        frame = detectFace(face_cascade=face_cascade, image=frame, gray_image=gray)
        frame = detectEdgesOffTheFace(image=frame, blur_image=img_blur, gray_image=gray, face_cascade=face_cascade)
        frame = detectMouth(detector=detector, predictor=predictor, image=frame, gray_image=gray)

        # Method 1: using a fusion algorithm, that uses the distance between top lip and bottom lip, speech detection
        # from audio and a timer for hysteresis (when we are speaking, our mouth closes as well)
        # https://livecodestream.dev/post/detecting-face-features-with-python/
        if isSpeaking(detector=detector, predictor=predictor, gray_image=gray) is True:
            cv2.putText(frame, 'Method 1: You are speaking', (40, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0))
        else:
            cv2.putText(frame, 'Method 1: Silence', (40, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255))

        # audio_data = stream.read(CHUNCK)
        # audio_rms = audioop.rms(audio_data, 2)

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

    # ---------------------------------------------------
    # Finalization
    # ---------------------------------------------------

    # Release everything if job is finished
    capture.release()
    cv2.destroyAllWindows()

    # # stop all streams and terminate pyaudio
    # stream.stop_stream()
    # stream.close()
    # p.terminate()


if __name__ == "__main__":
    main()
