import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from platforms_server.msg import FieldObjects as FieldObjects_msg, AllPathes
from vision.vision_constants import IMAGE_SIZE, MAP_ROWS, MAP_COLUMNS
# from vision.Fileds_objects import ImageMap
from vision.Fileds_objects import Point, ImageMap


class Visualizer():
    def __init__(self):
        self.bridge = CvBridge()
        self.IMG = None
        self.fields_objects = None
        self.pathes = None
        self.map = ImageMap()
        self.map.set_map_params(IMAGE_SIZE, IMAGE_SIZE, MAP_ROWS, MAP_COLUMNS)
        self.map.create_sectors()
        self.do_draw_map = False

        rospy.init_node("visualizer_node")
        img_sub = rospy.Subscriber("square_image", Image, self.img_callback)
        field_objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, self.objects_callback)

    def img_callback(self, data):
        if not rospy.is_shutdown():
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.IMG = cv_image
                self.draw_objects()
                self.draw_paths()
                self.clear_field_objects()
                self.clear_paths()
                if self.do_draw_map:
                    self.draw_map()
                    self.draw_sectors_num()
                cv2.imshow("img22", self.IMG)
                if cv2.waitKey(3) & 0xFF == ord("d"):
                    self.do_draw_map = not self.do_draw_map
            except CvBridgeError as e:
                print(e)

    def draw_point(self, point, color=(255, 0, 100), size=3):
        cv2.circle(self.IMG, (int(point.x), int(point.y)), 1, color, thickness=size)

    def objects_callback(self, data):
        self.fields_objects = data

    def clear_paths(self):
        self.pathes = None

    def clear_field_objects(self):
        self.fields_objects = None

    def draw_map(self):
        columns = self.map.column.values()
        rows = self.map.row.values()
        merge_list = lambda ll: [el for lst in ll for el in lst]
        columns = merge_list(columns)
        rows = merge_list(rows)
        for c in columns:
            for r in rows:
                self.draw_point(Point(c, r), color=(45, 205, 255), size=2)

    def draw_sectors_num(self):
        for c in range(self.map.columns_num):
            for r in range(self.map.rows_num):
                center = self.map.get_sector_center(r, c)
                cv2.putText(self.IMG, "({}, {})".format(r, c), (center.x-self.map.sector_w//2, center.y-self.map.sector_h//3),
                            cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 0, 0), thickness=1)

    def draw_objects(self):
        if self.fields_objects:
            for robot in self.fields_objects.robots:
                if not robot.on_finish:
                    self.draw_point(robot.center)
                    for pt in robot.path:
                        self.draw_point(pt, size=8)
                    self.draw_point(robot.direction, color=(50, 50, 250), size=5)
                    cv2.line(self.IMG, (int(robot.center.x), int(robot.center.y)),
                             (int(robot.direction.x), int(robot.direction.y)), color=(100, 100, 255), thickness=3)
                    cv2.putText(self.IMG, str(robot.id), (int(robot.center.x) + 5, int(robot.center.y)),
                                cv2.FONT_HERSHEY_PLAIN, 2,
                                (255, 100, 60), 3)
                    if robot.actual_point:
                        self.draw_crest(robot.actual_point, color=(100, 255, 50))
                        cv2.putText(self.IMG, str(robot.id), (int(robot.actual_point.x) + 5, int(robot.actual_point.y)), cv2.FONT_HERSHEY_PLAIN, 2,
                                    (255, 100, 60), 3)
                    if robot.next_point:
                        self.draw_crest(robot.next_point, color=(255, 50, 50))
                    if robot.sector:
                        if self.do_draw_map:
                            center = self.map.get_sector_center(*robot.sector)
                            cv2.circle(self.IMG, center.get_xy(), self.map.sector_h//2, (0, 0, 255), 2)
            for obstacle in self.fields_objects.obstacles:
                self.draw_point(obstacle.center)
            for goal in self.fields_objects.goals:
                self.draw_point(goal.center)

    def draw_crest(self, point, color=(0, 0, 255)):
        line_size = 10
        top = (int(point.x), int(point.y - line_size))
        bot = (int(point.x), int(point.y + line_size))
        left = (int(point.x) - line_size, int(point.y))
        right = (int(point.x) + line_size, int(point.y))
        cv2.line(self.IMG, top, bot, color, thickness=3)
        cv2.line(self.IMG, left, right, color, thickness=3)

    def draw_paths(self):
        if self.pathes:
            print(self.pathes)
            for path in self.pathes.paths_list:
                for pt in path.path_points:
                    self.draw_point(pt, size=5)


    def start_spin(self):
        rospy.spin()

