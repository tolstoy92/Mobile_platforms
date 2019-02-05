#!/usr/bin/env python


import rospy
import time
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
            topic = MAIN_TOPIC + str(robot.id)
            msg = str(robot.angle_to_actual_point)
            msg_sender.send_msg_with_delay(client, topic, msg)


rospy.init_node("mqtt_node")
fields_data_sub = rospy.Subscriber("field_objects", FieldObjects, mqtt_callback)
rospy.spin()





