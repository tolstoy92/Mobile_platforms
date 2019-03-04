#!/usr/bin/env python


import cv2

import rospy
from time import sleep
from random import randint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision.Fileds_objects import Point
from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg
from vision.Fileds_objects import ImageMap
from vision.vision_constants import IMAGE_SIZE, MAP_COLUMNS, MAP_ROWS

MARKER_IDS = [2, 3, 4]

path_points_num = 3 # to change points number!

map = ImageMap()
map.set_map_params(IMAGE_SIZE, IMAGE_SIZE, MAP_ROWS, MAP_COLUMNS)
map.create_sectors()

image = None
X, Y = None, None
sector_x , sector_y = None, None

bridge = CvBridge()


def img_callback(msg_data):
    global image
    image = bridge.imgmsg_to_cv2(msg_data, "bgr8")


def cv_callback(event,x,y,flags,param):
    global X, Y
    global sector_x, sector_y

    if event == cv2.EVENT_LBUTTONUP:
        X, Y = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        r, c = map.get_point_position_on_map(Point(x, y))
        point = map.get_sector_center(r, c)
        sector_x, sector_y = point.x, point.y


def draw_rectangle(center, width):
    points = []
    points.append((int(center[0] - width // 2), int(center[1] - width // 2)))
    points.append((int(center[0] + width // 2), int(center[1] - width // 2)))
    points.append((int(center[0] + width // 2), int(center[1] + width // 2)))
    points.append((int(center[0] - width // 2), int(center[1] + width // 2)))
    for i in range(len(points)):
        cv2.line(image, points[i], points[i-1], (255, 200, 40), 2)


cv2.namedWindow('image_to_set_path')
cv2.setMouseCallback('image_to_set_path', cv_callback)

final_msg = AllPathes()

def create_msg(id, path):
    path_msg = Path()
    path_msg.platform_id = id
    path_msg.path_points = path
    return path_msg

OK = False

was_robot_data = False

def obj_callback(msg_data):
    global was_robot_data
    if not was_robot_data:
        final_msg = AllPathes()
        global X, Y, OK
        for robot in msg_data.robots:
            if not robot.path_created:
                OK = False
                path = []
                while not OK:
                    cv2.putText(image, "Select sector. (You need {} points)".format(path_points_num), \
                                (20, 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 50, 40), 2)
                    cv2.circle(image, (int(robot.center.x), int(robot.center.y)), 5, (255, 255, 10), 4)
                    # if sector_x and sector_y:                                     # !!! uncomment for use sectors
                        # draw_rectangle((sector_x, sector_y), map.sector_w)        # !!! uncomment for use sectors
                    if len(path):
                        for pt in path:
                            cv2.circle(image, (int(pt.x), int(pt.y)), 5, (255, 100, 50), 4)
                    cv2.imshow("image_to_set_path", image)
                    cv2.waitKey(100)
                    if X and Y:
                        path_point = map.get_point_position_on_map(Point(X, Y))
                        cv_path_point = map.get_sector_center(*path_point)
                        if len(path) < path_points_num:
                            path.append(Point(X, Y))                                # !!! comment for use sectors
                            # path.append(cv_path_point)                            # !!! uncomment for use sectors
                            X, Y = None, None
                        else:
                            OK = not OK
                            X, Y = None, None
                msg = create_msg(robot.id, path)
                final_msg.paths_list.append(msg)
            else:
                OK = True
        paths_data_publisher.publish(final_msg)
        cv2.destroyAllWindows()
        was_robot_data = True



rospy.init_node("test_set_path")
img_sub = rospy.Subscriber("square_image", Image, img_callback)
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, obj_callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)


rospy.spin()
