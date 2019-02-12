#!/usr/bin/env python


import rospy
import cv2
from cv2 import aruco
from platforms_server.msg import ArucoData, MarkerData, Point2d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def corners_to_msg(corners):
    corners_msgs_list = []
    for corner in corners:
        corners_msg = list(Point2d(x, y) for x, y in corner[0])
        corners_msgs_list.append(corners_msg)
    return corners_msgs_list


def callback(data):
    if not rospy.is_shutdown():
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            all_corners, ids_np, _ = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
            if not isinstance(ids_np, type(None)):
                aruco_data_msg = ArucoData()
                corners_msgs_list = corners_to_msg(all_corners)
                ids_msgs_list = list(id[0] for id in ids_np.tolist())
                markers = list(MarkerData(id, corners) for id, corners in zip(ids_msgs_list, corners_msgs_list))
                aruco_data_msg.markers = markers
                markers_data_publisher.publish(aruco_data_msg)

        except CvBridgeError as e:
            print(e)
    else:
        cv2.destroyAllWindows()


bridge = CvBridge()

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()

rospy.init_node("aruco_detector_node")
markers_data_publisher = rospy.Publisher("detected_markers", ArucoData)
image_sub = rospy.Subscriber("square_image", Image, callback)

rospy.spin()

