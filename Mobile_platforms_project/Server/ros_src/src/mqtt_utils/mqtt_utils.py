from random import randint
import time


class MqttClientTools():
    def __init__(self, server_ip, port, qos, connection_topic, delay_time):
        self.last_sending_time = time.time()
        self.qos = qos
        self.server_ip = server_ip
        self.port = port
        self.connection_topic = connection_topic
        self.delay_time = delay_time


    def wait_for(self, client, msgType, period=0.25):
        if msgType == "SUBACK":
            if client.on_subscribe:
                while not client.suback_flag:
                    print("waiting suback")
                    client.loop()  # check for messages
                    time.sleep(period)

    def start_connection_client(self, client):
        client.on_connect = self.on_connect
        client.on_disconnect = self.on_disconnect
        client.on_message = self.on_message
        client.connect(self.server_ip, self.port, keepalive=6)
        client.connect_topic = self.connection_topic
        client.subscribe(self.connection_topic, qos=2)

    def on_connect(self, client, user_data, flags, rc):
        client.loop_start()
        if rc == 0:
            client.connected_flag = True  # set flag
            print("connected OK")
        else:
            print("Bad connection Returned code=", rc)

    def on_message(self, client, user_data, message):
        if message.topic == client.connect_topic:
            print("Platform %s is connected." % str(message.payload.decode("utf-8")))
        print("message: ", str(message.payload.decode("utf-8")))
        print("message topic: ", message.topic)
        print("message qos: ", message.qos)
        print("message retain flag: ", message.retain)

    def on_disconnect(self, client, user_rdata, rc=0):
        print("DisConnected result code " + str(rc))
        client.loop_stop()
        client.disconnect()

    def send_rand_msg(self, client, platform_lst, msg_topic):
        for platform in platform_lst:
            msg = "Test msg " + str(randint(50, 100))
            topic = msg_topic + str(platform)
            client.publish(topic, msg, qos=self.qos)

    def send_msg(self, client, topic, msg):
        msg = str(msg)
        client.publish(topic, msg, qos=self.qos)

    def update_last_sendig_time(self):
        self.last_sending_time = time.time()

    def send_msg_with_delay(self, client, topic, msg):
        delay = time.time() - self.last_sending_time
        if delay > self.delay_time:
            self.send_msg(client, topic, msg)  # args = client, topic, msg, qos
            self.update_last_sendig_time()

    def send_multiple_msg_with_delay(self, client, topics_lst, msgs_lst):
        delay = time.time() - self.last_sending_time
        if delay > self.delay_time:
            l = list(zip(topics_lst, msgs_lst))
            for topic, msg in l:
                self.send_msg(client, topic, msg)
            self.update_last_sendig_time()
