#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#program to do static landing
current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def set_position(x, y, z, local_position_pub):

    # Function to set the desired position of the UAV
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    return pose

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        detect_aruco(cv_image)
    except Exception as e:
        print(e)

def detect_aruco(image):
    
    # ArUco detection logic

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

        # Estimate the pose of each detected ArUco marker
        # rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 1.0, camera_matrix, dist_coeffs)

        # Draw the marker outlines and axes for each detected marker
        for i in range(len(ids)):
            # aruco.drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 1.0)
            aruco.drawDetectedMarkers(image, corners, ids)

        # Land if the ArUco marker is close enough
        # if corners[0][0][0][2] < 0.1:
        #     rospy.loginfo("ArUco marker detected. Landing...")

        #     offb_set_mode.custom_mode = 'AUTO.LAND'
        #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #         rospy.loginfo("AUTO.LAND enabled")

    # Display the result
    cv2.imshow("ArUco Detection", image)
    cv2.waitKey(1)

if __name__ == "__main__":

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    image_raw_sub = rospy.Subscriber('/stereo/right/image_raw', Image, image_callback)
    local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    init_x = 0
    init_y = 0
    takeoff_altitude = 5
    pose = set_position(init_x, init_y, takeoff_altitude, local_pos_pub)

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):

            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()

        elif(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):

            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")

            last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        if abs(current_pose.pose.position.z - takeoff_altitude) <=  0.1:

            # Set the desired navigation point
            target_x = 7.0
            target_y = 7.0
            target_z = takeoff_altitude  # Maintain the same altitude for simplicity

            # Set the UAV to the desired position
            new_pose = set_position(target_x, target_y, target_z, local_pos_pub)
            rospy.loginfo("Navigating to target position")

            # Wait for the UAV to reach the target position
            while not rospy.is_shutdown() and \
                    abs(current_pose.pose.position.x - target_x) > 0.1 or \
                    abs(current_pose.pose.position.y - target_y) > 0.1:
                rospy.sleep(0.1)
        
        rate.sleep()

