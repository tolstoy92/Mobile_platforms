#!/usr/bin/env python


import rospy
from time import sleep
from random import randint
from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg
from vision.Fileds_objects import ImageMap
from vision.vision_constants import IMAGE_SIZE, MAP_COLUMNS, MAP_ROWS


MARKER_IDS = [2, 3, 4]

map = ImageMap()
map.set_map_params(IMAGE_SIZE, IMAGE_SIZE, MAP_ROWS, MAP_COLUMNS)
map.create_sectors()

path_points_num = 5000

final_msg = AllPathes()


def generate_random_sector(rows_num, columns_num):
    r = randint(1, rows_num-2)
    c = randint(1, columns_num-2)
    point = map.get_sector_center(r, c)
    return point

def generate_path(rows_num, columns_num):
    path = []
    for x in range(path_points_num):
        path.append(generate_random_sector(rows_num, columns_num))
    return path

def create_msg(id):
    path = generate_path(MAP_ROWS, MAP_COLUMNS)
    path_msg = Path()
    path_msg.platform_id = id
    path_msg.path_points = path
    return path_msg

def callback(msg_data):
    final_msg = AllPathes()
    for robot in msg_data.robots:
        msg = create_msg(robot.id)
        final_msg.paths_list.append(msg)
    paths_data_publisher.publish(final_msg)

rospy.init_node("test_sectors_path_generating", anonymous=True)
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)

rospy.spin()



