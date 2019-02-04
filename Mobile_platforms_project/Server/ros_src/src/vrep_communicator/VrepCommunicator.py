import vrep
import vrep_constants as const
import math
from matplotlib.path import Path
from numpy import array

class Point:
    def __init__(self, xy=None):
        if isinstance(xy, tuple):
            if len(xy) >= 2:
                self.x = xy[0]
                self.y = xy[1]
        else:
            if not isinstance(xy, type(None)):
                self.x = xy.x
                self.y = xy.y

    def __str__(self):
        return str(self.x) + " " + str(self.y)

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_xy(self, x, y):
        self.x = x
        self.y = y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_xy(self):
        return self.x, self.y

class Vrep():
    def __init__(self):
        self.client_id = vrep.simxStart(const.CON_ADDRESS, const.CON_PORT, False, True, \
                                        const.TIMEOUT_IN_MS, const.COMM_THREAD_CYCLE_IN_MS)

    def get_object_handle(self, obj_name):
        ret, handle = vrep.simxGetObjectHandle(self.client_id, obj_name, vrep.simx_opmode_oneshot_wait)
        return handle

    def get_object_child(self, parent_handle, index):
        ret, child_handle = vrep.simxGetObjectChild(self.client_id, \
                            parent_handle, index, vrep.simx_opmode_oneshot_wait)
        return child_handle

    def get_object_position(self, object_handle):
        """
        Function that returns position of object on the scene in V-REP
        """
        res, object_position = vrep.simxGetObjectPosition(self.client_id, object_handle, -1, \
                                                          vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return Point((object_position[0], object_position[1]))
        else:
            print('Remote function call failed with result {0}.'.format(res))
            return ()

    def get_robots_data(self):
        robots_data = dict()
        robots_handles = self.get_object_childs(const.ROBOTS_NAMES_TREE)
        for robot in robots_handles:
            robot_boundary_points = self.get_boundary_points(robot)
            robot_position = self.get_object_position(robot)
            robot_direction = self.get_robot_direction_vector(robot)
            robots_data[robot] = [robot_position, robot_direction, robot_boundary_points]
        return robots_data

    def get_goal_data(self):
        goal_data = dict()
        goal_handles = self.get_object_childs(const.TARGETS_NAMES_TREE)
        for goal in goal_handles:
            goal_boundary_points = self.get_boundary_points(goal)
            goal_position = self.get_object_position(goal)
            goal_data[goal] = [goal_position, goal_boundary_points]
        return goal_data

    def get_obstacles_data(self):
        if const.WITH_DYNAMIC_OBSTACLES:
            pass
        else:
            obstacles_data = dict()
            obstacle_handles = self.get_object_childs(const.OBSTACLES_NAMES_TREE)
            for obstacle in obstacle_handles:
                obstacle_boundary_points = self.get_boundary_points(obstacle)
                obstacle_position = self.get_object_position(obstacle)
                obstacles_data[obstacle] = [obstacle_position, obstacle_boundary_points]
            return obstacles_data

    def get_boundary_points(self, object_handle):
        """
        Function that returns boundary points of object's (obstacle) boundary box
        """
        points = []
        obstacle_position = self.get_object_position(object_handle)
        ret, orient = vrep.simxGetObjectOrientation(self.client_id, object_handle, -1, \
                                                    vrep.simx_opmode_blocking)
        ret, x_1 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 15, \
                                                    vrep.simx_opmode_blocking)
        ret, y_1 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 16, \
                                                    vrep.simx_opmode_blocking)
        ret, x_2 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 18, \
                                                    vrep.simx_opmode_blocking)
        ret, y_2 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 19, \
                                                    vrep.simx_opmode_blocking)
        angle = orient[2]
        # Extension of boundaries, so that the robots moves without collisions
        x_1 = x_1 - 0.3
        x_2 = x_2 + 0.3
        y_1 = y_1 - 0.3
        y_2 = y_2 + 0.3


        p_1 = (x_1 * math.cos(angle) - y_1 * math.sin(angle) + obstacle_position.x, y_1 * \
               math.cos(angle) + x_1 * math.sin(angle) + obstacle_position.y)
        points.append(Point(p_1))
        p_2 = (x_1 * math.cos(angle) - y_2 * math.sin(angle) + obstacle_position.x, y_2 * \
               math.cos(angle) + x_1 * math.sin(angle) + obstacle_position.y)
        points.append(Point(p_2))
        p_3 = (x_2 * math.cos(angle) - y_2 * math.sin(angle) + obstacle_position.x, y_2 * \
               math.cos(angle) + x_2 * math.sin(angle) + obstacle_position.y)
        points.append(Point(p_3))
        p_4 = (x_2 * math.cos(angle) - y_1 * math.sin(angle) + obstacle_position.x, y_1 * \
               math.cos(angle) + x_2 * math.sin(angle) + obstacle_position.y)
        points.append(Point(p_4))
        return points

    def get_object_childs(self, obj_name):
        """
        Function that return handles of object's childs from the V-REP scene.
        This function is useful when the exact number of objects is unknown
        """
        index = 0
        children_list = []
        child = 0
        parent_handle = self.get_object_handle(obj_name)
        while child != -1:
            res, child = vrep.simxGetObjectChild(self.client_id, parent_handle, index, vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                children_list.append(child)
                index = index + 1
            else:
                print('Remote fucntion get_object_childs call failed.')
                return []
        del children_list[len(children_list) - 1]
        return children_list

    def finish_connection(self):
        vrep.simxFinish(-1)

    def get_robot_direction_vector(self, robot_handle):
        direction_point = self.get_object_child(robot_handle, 15)
        robot_position = self.get_object_position(robot_handle)
        dir_point_position = self.get_object_position(direction_point)
        direction_vector = (dir_point_position.x - robot_position.x, \
                            dir_point_position.y - robot_position.y)
        direction_vector_mod = math.sqrt(direction_vector[0] ** 2 \
                                         + direction_vector[1] ** 2)
        norm_direction_vector = (direction_vector[0] / direction_vector_mod, \
                                 direction_vector[1] / direction_vector_mod)
        return Point(norm_direction_vector)

