#!/usr/bin/env python


import rospy
import paho.mqtt.client as mqtt
from mqtt_utils.mqtt_utils import MqttClientTools
from mqtt_utils.mqtt_constants import *
from platforms_server.msg import ArucoData, MarkerData, Point2d, FieldObjects


client = mqtt.Client("Server")
msg_sender = MqttClientTools(SERVER_IP, PORT, MESSAGES_QOS, CONNECTON_TOPIC, DELAY_TIME)


msg_sender.start_connection_client(client, )
client.subscribe(FEEDBACK_TOPIC, qos=MESSAGES_QOS)
client.loop(WORKER_TIME)


def mqtt_callback(msg_data):
    for robot in msg_data.robots:
        if robot.path_created:
            topics_list, msgs_list = [], []

            platform_topic = MAIN_TOPIC + str(robot.id)

            angle_topic = platform_topic + "/angle"
            angle_msg = str(robot.actual_angle)
            topics_list.append(angle_topic)
            msgs_list.append(angle_msg)

            move_topic = platform_topic + "/move"
            if robot.move:
                move_msg = "1"
            else:
                move_msg = "0"
            topics_list.append(move_topic)
            msgs_list.append(move_msg)

            rotation_topic = platform_topic + "/rotation"
            if robot.rotation:
                rotation_msg = "1"
            else:
                rotation_msg = "0"
            topics_list.append(rotation_topic)
            msgs_list.append(rotation_msg)

            finish_topic = platform_topic + "/on_finish"
            if robot.on_finish:
                finish_msg = "1"
            else:
                finish_msg = "0"
            topics_list.append(finish_topic)
            msgs_list.append(finish_msg)

            msg_sender.send_multiple_msg_with_delay(client, topics_list, msgs_list)


rospy.init_node("mqtt_node")
fields_data_sub = rospy.Subscriber("field_objects", FieldObjects, mqtt_callback)
rospy.spin()





