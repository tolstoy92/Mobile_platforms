#!/usr/bin/env python


import rospy
from time import sleep
from random import randint
from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg
from vision.Fileds_objects import ImageMap
from vision.vision_constants import IMAGE_SIZE, MAP_COLUMNS, MAP_ROWS


MARKER_ID = 2


map = ImageMap()
map.set_map_params(IMAGE_SIZE, IMAGE_SIZE, MAP_ROWS, MAP_COLUMNS)
map.create_sectors()


def generate_random_sector(rows_num, columns_num):
    r = randint(0, rows_num-1)
    c = randint(0, columns_num-1)
    point = map.get_sector_center(r, c)
    return point


path = []
path_points_num = 4


for x in range(path_points_num):
    path.append(generate_random_sector(map.rows_num, map.columns_num))


path_msg = Path()
path_msg.platform_id = MARKER_ID
path_msg.path_points = path

final_msg = AllPathes()
final_msg.paths_list = []
final_msg.paths_list.append(path_msg)

print(final_msg)


def callback(msg_data):
    # pass
    # print("a")
    paths_data_publisher.publish(final_msg)
    # print("b")

rospy.init_node("test_sectors_path_generating", anonymous=True)
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=5)

rospy.spin()



