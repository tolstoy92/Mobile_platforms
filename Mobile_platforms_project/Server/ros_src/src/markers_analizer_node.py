#!/usr/bin/env python

import rospy
from platforms_server.msg import ArucoData, FieldObjects
from vision.MarkersAnalizer import MarkersAnalizer


def recognize_fields_object_by_id(msg_data):
    ids = []
    corners = []
    for object in msg_data.markers:
        ids.append(object.id)
        corners.append(object.corners)
    markers_dict = dict(zip(ids, corners))
    analizer.parse_fields_objects_by_id(markers_dict)
    robots = analizer.get_robots()
    goals = analizer.get_goals()
    obstacles = analizer.get_obstacles()

    return robots, goals, obstacles


def callback(msg_data):
    objects_msg = FieldObjects()
    robots, goals, obstacles = recognize_fields_object_by_id(msg_data)

    objects_msg.robots = list(robot.prepare_msg() for robot in robots.values())
    objects_msg.goals = list(goal.prepare_msg() for goal in goals.values())
    objects_msg.obstacles = list(obstacle.prepare_msg() for obstacle in obstacles.values())

    field_objects_pub.publish(objects_msg)


analizer = MarkersAnalizer()
rospy.init_node("markers_analizer_node")
markers_data_sub = rospy.Subscriber("detected_markers", ArucoData, callback)
field_objects_pub = rospy.Publisher("field_objects", FieldObjects)


rospy.spin()
