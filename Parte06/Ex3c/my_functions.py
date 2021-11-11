#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import time
import cv2
from scipy.spatial import distance

# ---------------------------------------------------
# Global Variables
# ---------------------------------------------------
tic = time.time()
dist, y, h, x, w = 0, 0, 0, 0, 0


# ------------------------
## FUNCTION DEFINITION ##
# ------------------------
def detectFace(face_cascade, image, gray_image):
    # create a copy of the frame for transparency process
    overlay = image.copy()

    # Detect faces in gray image
    faces_opencv = face_cascade.detectMultiScale(gray_image, 1.1, 5)

    # Draw filled rectangle around the faces and change transparency
    for (x, y, w, h) in faces_opencv:
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), -1)
        alpha = 0.15  # Transparency factor.
        # Following line overlays transparent rectangle over the image
        image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    return image


def detectEdgesOffTheFace(image, blur_image, gray_image, face_cascade):
    # Canny Edge Detection
    global y, h, x, w
    edges = cv2.Canny(image=blur_image, threshold1=100, threshold2=200)

    mask_edges = edges.astype(bool)  # Convert the edges from uint8 to boolean

    # Detect faces in gray image
    faces = face_cascade.detectMultiScale(gray_image, 1.1, 5)

    for (x, y, w, h) in faces:
        # Where the rectangle should be, put False on mask
        mask_edges[y:y + h, x:x + w] = False

    # Where the rectangle should be, put False on mask
    mask_edges[y:y + h, x:x + w] = False

    # Change the pixels where we have edges to red.
    image[mask_edges] = (0, 0, 255)  # Where the mask is true, change the pixels to red

    return image


def detectMouth(detector, predictor, image, gray_image):
    # Use detector to find landmarks for the speaking in gray image
    faces_dlib = detector(gray_image)

    # Draw lines around the mouth, or circles in each point of the mouth
    for face in faces_dlib:
        # Create landmark object
        landmarks = predictor(image=gray_image, box=face)

        # Loop through all the points
        for n in range(48, 59):
            (x1, y1) = landmarks.part(n).x, landmarks.part(n).y
            (x2, y2) = landmarks.part(n + 1).x, landmarks.part(n + 1).y
            # cv2.circle(img=frame, center=(x1, y1), radius=1, color=(255, 0, 0), thickness=-1)
            cv2.line(img=image, pt1=(x1, y1), pt2=(x2, y2), color=(255, 0, 0))
        # Complete the mouth
        (x1, y1) = landmarks.part(48).x, landmarks.part(48).y
        (x2, y2) = landmarks.part(59).x, landmarks.part(59).y
        cv2.line(img=image, pt1=(x1, y1), pt2=(x2, y2), color=(255, 0, 0))

    return image


def isSpeaking(detector, predictor, gray_image):
    # initial setup
    global dist, tic

    # Use detector to find landmarks for the speaking in gray image
    faces_dlib = detector(gray_image)

    # Get coordinates of the top lip and the bottom lip, and get euclidean distance between both
    for face in faces_dlib:
        landmarks = predictor(image=gray_image, box=face)
        (x1, y1) = landmarks.part(62).x, landmarks.part(62).y
        (x2, y2) = landmarks.part(66).x, landmarks.part(66).y
        dist = distance.euclidean([x1, y1], [x2, y2])

    # Check the distance calculated before to see if the person is speaking or not
    if dist > 6.2:
        tic = time.time()
        return True
    elif time.time() - tic < 0.5:
        return True
    else:
        return False
