#!/usr/bin/env python

import rospy
from vrep_communicator.VrepCommunicator import Vrep, Robot
import vrep_communicator.vrep_constants as const
from platforms_server.msg import RobotData, WheelRotationData, XML_PATH, Step
from collections import namedtuple

def prepare_robot_msg(robots):
    goal_msgs = []
    for id in robots:
        msg = RobotData()
        msg.id = id
        msg.position = robots[id][0]
        msg.direction = robots[id][1]
        msg.corners = robots[id][2]
        goal_msgs.append(msg)
    return goal_msgs

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

def callback(msg_data):
    robot_id = msg_data.id
    left_velo = msg_data.left_velocity
    right_velo = msg_data.right_velocity
    robot_thread.wheel_rotation(left_velo, right_velo)
    global path_msg
    if msg_data.goal_reached:
        path_msg.path.pop(0)
    msg = RobotData()
    if path_msg.path:
        msg.id = robot_id
        msg.position = robot_thread.get_robot_position()
        msg.direction = robot_thread.get_robot_orientation()
        msg.path = path_msg
        msg.actual_point = path_msg.path[0]
        msg.angle_to_actual_point = vrep_con.get_object_orientation(robot_id, path_msg.path[0].finish)
        msg.rotation = msg_data.rotation
        msg.on_finish = False
    else:
        msg.on_finish = True
        vrep_con.end_simulation()
    robot_data_pub.publish(msg)

rospy.init_node("vrep_communicator_node")
robot_data_pub = rospy.Publisher("robot_movement_data", RobotData)
wheel_rotation_data = rospy.Subscriber("wheel_rotation_data", WheelRotationData, callback)
rows = 15
cols = 15

Step_tup = namedtuple("step", ["number",
                           "start",
                           "finish",
                           "duration"])

vrep_con = Vrep(const.CON_PORT)

robots_data = vrep_con.get_robots_data()
cell_mas = vrep_con.create_mesh(rows, cols)

step_0 = Step_tup(0, cell_mas[4][9], cell_mas[4][7], 2.)
step_1 = Step_tup(1, cell_mas[4][7], cell_mas[4][7], 2.885337)
step_2 = Step_tup(2, cell_mas[4][7], cell_mas[8][7], 4.)
step_3 = Step_tup(3, cell_mas[8][7], cell_mas[8][7], 0.204833)
step_4 = Step_tup(4, cell_mas[8][7], cell_mas[11][6], 3.162278)

path = [step_0, step_1, step_2, step_3, step_4]
path_msg = prepare_path_msg(path)
for step in path:
    if not step.start == step.finish:
        point_pos = step.finish
        vrep_con.create_dummy(point_pos)

robots = prepare_robot_msg(robots_data)
robot = robots[0]

port_num = const.CON_PORT + 1
robot_thread = Robot(robot.id, port_num)

msg = RobotData()
msg.id = robot.id
msg.position = robot_thread.get_robot_position()
msg.direction = robot_thread.get_robot_orientation()
msg.path = path_msg
msg.actual_point = path_msg.path[0]
msg.angle_to_actual_point = vrep_con.get_object_orientation(robot.id, path_msg.path[0].finish)
msg.on_finish = False
msg.rotation = True
robot_data_pub.publish(msg)

rospy.spin()
