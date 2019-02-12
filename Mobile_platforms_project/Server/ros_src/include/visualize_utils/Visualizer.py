import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from platforms_server.msg import FieldObjects as FieldObjects_msg, AllPathes

class Visualizer():
    def __init__(self):
        self.bridge = CvBridge()
        self.IMG = None
        self.fields_objects = None
        self.pathes = None
        rospy.init_node("visualizer_node")
        img_sub = rospy.Subscriber("square_image", Image, self.img_callback)
        field_objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, self.objects_callback)
        path_sub = rospy.Subscriber("paths_data", AllPathes, self.paths_callback)

    def img_callback(self, data):
        if not rospy.is_shutdown():
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.IMG = cv_image
                self.draw_objects()
                self.draw_paths()
                cv2.imshow("img22", self.IMG)
                cv2.waitKey(3)
            except CvBridgeError as e:
                print(e)

    def draw_point(self, point):
        cv2.circle(self.IMG, (int(point.x), int(point.y)), 1, (255, 0, 100), 3)

    def objects_callback(self, data):
        self.fields_objects = data

    def draw_objects(self):
        if self.fields_objects:
            for robot in self.fields_objects.robots:
                self.draw_point(robot.center)
            for obstacle in self.fields_objects.obstacles:
                self.draw_point(obstacle.center)
            for goal in self.fields_objects.goals:
                self.draw_point(goal.center)

    def paths_callback(self, data):
        self.pathes = data

    def draw_paths(self):
        if self.pathes:
            for path in self.pathes.paths_list:
                for pt in path.path_points:
                    self.draw_point(pt)

    def start_spin(self):
        rospy.spin()