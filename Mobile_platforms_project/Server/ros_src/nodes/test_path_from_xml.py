#!/usr/bin/env python


import rospy
from collections import namedtuple
from platforms_server.msg import XML_PATH, Step


Point = namedtuple("point", ["x", "y"])
Step_tup = namedtuple("step", ["number",
                           "start",
                           "finish",
                           "duration"])

step_0 = Step_tup(0, Point(4, 9), Point(4, 7), 2.)
step_1 = Step_tup(1, Point(4, 7), Point(4, 7), 2.885337)
step_2 = Step_tup(2, Point(4, 7), Point(8, 7), 4.)
step_3 = Step_tup(3, Point(8, 7), Point(8, 7), 0.204833)
step_4 = Step_tup(4, Point(8, 7), Point(11, 6), 3.162278)

path = [step_0, step_1, step_2, step_3, step_4]


def prepare_path_msg(step_list):
    path_msg = XML_PATH()
    path_msg.path = []
    for step in step_list:
        path_msg.path.append(prepare_step_msg(step))
    return path_msg

def prepare_step_msg(step):
    step_msg = Step()
    step_msg.number = step.number
    step_msg.start = step.start
    step_msg.finish = step.finish
    step_msg.duration = step.duration
    return step_msg

msg = prepare_path_msg(path)

rospy.init_node("path_from_xml")
pub = rospy.Publisher("path_from_xml", XML_PATH, queue_size=1)

while not rospy.is_shutdown():
    pub.publish(msg)
