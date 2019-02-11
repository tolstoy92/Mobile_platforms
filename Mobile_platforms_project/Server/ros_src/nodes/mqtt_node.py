#!/usr/bin/env python


import rospy
import paho.mqtt.client as mqtt
from mqtt_utils.mqtt_utils import MqttClientTools
from mqtt_utils.mqtt_constants import *
from platforms_server.msg import ArucoData, MarkerData, Point2d, FieldObjects

client = mqtt.Client("Server")
msg_sender = MqttClientTools(SERVER_IP, PORT, MESSAGES_QOS, CONNECTON_TOPIC, DELAY_TIME)

msg_sender.start_connection_client(client, )
client.subscribe(FEEDBACK_TOPIC, qos=2)
client.loop(WORKER_TIME)


def prepare_angle_msg(angle):
    if angle < 0:
        sign = "0"
    else:
        sign = "1"
    angle_msg = str(abs(angle))
    if len(angle_msg) == 3:
        msg = sign + angle_msg
    elif len(angle_msg ) == 2:
        msg = sign + "0" + angle_msg
    elif len(angle_msg) == 1:
        msg = sign + "00" + angle_msg
    return msg


def prepare_move_msg(move):
    if move == 0:
        msg = "0"
    else:
        msg = "1"
    return msg


def prepare_rotation(rot):
    if rot == 0:
        msg = "0"
    else:
        msg = "1"
    return  msg


def prepare_finisg_msg(finish):
    if finish == 0:
        msg = "0"
    else:
        msg = "1"
    return msg


def mqtt_callback(msg_data):
    for robot in msg_data.robots:
        if robot.path_created:
            topics_list, msgs_list = [], []

            platform_topic = MAIN_TOPIC + str(robot.id)

            angle_msg = prepare_angle_msg(robot.actual_angle)
            move_msg = prepare_move_msg(robot.move)
            rotate_msg = prepare_rotation(robot.rotation)
            finish_msg = prepare_finisg_msg(robot.on_finish)

            final_msg = angle_msg + move_msg + rotate_msg + finish_msg


            msg_sender.delay_time = DELAY_TIME
            msg_sender.send_msg_with_delay(client, platform_topic, final_msg)

            #
            # platform_topic = MAIN_TOPIC + str(robot.id)
            #
            # angle_topic = platform_topic + "/angle"
            # angle_msg = str(robot.actual_angle)
            # topics_list.append(angle_topic)
            # msgs_list.append(angle_msg)
            #
            # move_topic = platform_topic + "/move"
            # if robot.move:
            #     move_msg = "1"
            # else:
            #     move_msg = "0"
            # topics_list.append(move_topic)
            # msgs_list.append(move_msg)
            #
            # rotation_topic = platform_topic + "/rotate"
            # if robot.rotation:
            #     rotation_msg = "1"
            # else:
            #     rotation_msg = "0"
            # topics_list.append(rotation_topic)
            # msgs_list.append(rotation_msg)
            #
            # finish_topic = platform_topic + "/on_finish"
            # if robot.on_finish:
            #     finish_msg = "1"
            # else:
            #     finish_msg = "0"
            # topics_list.append(finish_topic)
            # msgs_list.append(finish_msg)
            #
            # if robot.rotation:
            #     msg_sender.delay_time = 0
            # else:
            #     msg_sender.delay_time = 0
            # msg_sender.send_multiple_msg_with_delay(client, topics_list, msgs_list)


rospy.init_node("mqtt_node")
fields_data_sub = rospy.Subscriber("field_objects", FieldObjects, mqtt_callback)
rospy.spin()





