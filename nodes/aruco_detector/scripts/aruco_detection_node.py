#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        detect_aruco(cv_image)
    except Exception as e:
        print(e)

def detect_aruco(image):
    # Your ArUco detection logic here
    # Use the aruco library functions provided by OpenCV

    # Example:
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # Initialize the ArUco detector
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, detectorParams=parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    # Draw markers on the image
    if ids is not None:
        aruco.drawDetectedMarkers(image, corners, ids)

    # Display the result
    cv2.imshow("ArUco Detection", image)
    cv2.waitKey(1)

def main():
    rospy.init_node('aruco_detection_node', anonymous=True)
    rospy.Subscriber('/stereo/right/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
