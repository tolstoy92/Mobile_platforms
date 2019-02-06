import rospy
from vision.Fileds_objects import Robot, Goal, Obstacle, Marker
from vision.vision_constants import EPS
from math import sqrt
from platforms_server.msg import FieldObjects as FieldObjects_msg, ArucoData, AllPathes


class MarkersAnalizer:
    def __init__(self):
        self.robots = {}
        self.goals = {}
        self.obstacles = {}
        rospy.init_node("markers_analizer_node")
        self.markers_data_sub = rospy.Subscriber("detected_markers", ArucoData, self.field_objects_callback)
        self.paths_data_sub = rospy.Subscriber("paths_data", AllPathes, self.paths_callback)
        self.field_objects_pub = rospy.Publisher("field_objects", FieldObjects_msg, queue_size=30)

    def recognize_fields_object_by_id(self, msg_data):
        ids = []
        corners = []
        for object in msg_data.markers:
            ids.append(object.id)
            corners.append(object.corners)
        markers_dict = dict(zip(ids, corners))
        self.parse_fields_objects_by_id(markers_dict)
        robots = self.get_robots()
        goals = self.get_goals()
        obstacles = self.get_obstacles()

        return robots, goals, obstacles

    def field_objects_callback(self, msg_data):
        self.clear_obstacles()
        objects_msg = FieldObjects_msg()
        robots, goals, obstacles = self.recognize_fields_object_by_id(msg_data)
        objects_msg.robots = list(robot.prepare_msg() for robot in robots.values())
        objects_msg.goals = list(goal.prepare_msg() for goal in goals.values())
        objects_msg.obstacles = list(obstacle.prepare_msg() for obstacle in obstacles.values())
        self.field_objects_pub.publish(objects_msg)

    def paths_callback(self, msg_data):
        paths_dict = {}
        for path in msg_data.paths_list:
            paths_dict[path.platform_id] = path.path_points
        for id in paths_dict:
            self.robots[id].set_path(paths_dict[id])

    def get_robots(self):
        return self.robots

    def get_goals(self):
        return self.goals

    def get_obstacles(self):
        return self.obstacles

    def set_pathes(self, pathes):
        self.pathes = pathes

    def parse_fields_objects_by_id(self, objects_dict):
        tmp_obstacles_dict = {}
        tmp_goals_dict = {}
        for key in objects_dict.keys():
            if len(str(key)) == 1:
                if key not in self.robots.keys():
                    self.robots[key] = Robot(key, objects_dict[key])
                else:
                    self.robots[key].update_data(objects_dict[key])
            elif len(str(key)) == 3:
                tmp_goals_dict[key] = Goal(key, objects_dict[key])
            else:
                if key//10 not in tmp_obstacles_dict:
                    tmp_obstacles_dict[key//10] = [Marker(key//10, objects_dict[key])]
                else:
                    tmp_obstacles_dict[key // 10].append(Marker(key//10, objects_dict[key]))
        for key in tmp_obstacles_dict.keys():
            self.obstacles[key] = Obstacle(key, tmp_obstacles_dict[key])

        if len(self.robots.keys()):
            self.set_goals_id_from_platform_id(tmp_goals_dict)
        else:
            self.goals = tmp_goals_dict

    def set_goals_id_from_platform_id(self, goals_dict):
        for (platform_id, goal_id) in list(zip(self.robots.keys(), goals_dict.keys())):
            self.goals[platform_id] = goals_dict[goal_id]

    def on_position(self, robot_position, target_position):
        on_target_point = False
        distance = self.get_distance_between_pts(robot_position, target_position)
        if distance <= EPS: on_target_point = True
        return on_target_point

    def get_distance_between_pts(self, pt1, pt2):
        return sqrt((pt2.x - pt1.x) ** 2 + (pt2.y - pt1.y) ** 2)

    def clear_robots(self):
        self.robots = {}

    def clear_goals(self):
        self.goals = {}

    def clear_obstacles(self):
        self.obstacles = {}

    def clear_fields_objects(self):
        self.clear_goals()
        self.clear_robots()
        self.clear_obstacles()
