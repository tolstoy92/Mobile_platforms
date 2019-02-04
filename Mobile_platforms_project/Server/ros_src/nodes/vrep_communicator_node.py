#!/usr/bin/env python

import rospy
from vrep_communicator.VrepCommunicator import Vrep
from platforms_server.msg import RobotData, GoalData, ObstacleData, FieldObjects


def prepare_robot_msg(robots):
    goal_msgs = []
    for id in robots:
        msg = RobotData()
        msg.id = id
        msg.center = robots[id][0]
        msg.direction = robots[id][1]
        msg.corners = robots[id][2]
        goal_msgs.append(msg)
    return goal_msgs


def prepare_goal_msg(goals):
    robot_msgs = []
    for id in goals:
        msg = GoalData()
        msg.id = id
        msg.center = goals[id][0]
        msg.corners = goals[id][1]
        robot_msgs.append(msg)
    return robot_msgs


def prepare_obstacle_msg(obstacles):
    obstacle_msgs = []
    for id in obstacles:
        msg = ObstacleData()
        msg.id = id
        msg.center = obstacles[id][0]
        msg.corners = obstacles[id][1]
        obstacle_msgs.append(msg)
    return obstacle_msgs


rospy.init_node("vrep_communicator_node")
vrep_data_publisher = rospy.Publisher("field_objects", FieldObjects)
vrep_con = Vrep()
objects_msg = FieldObjects()
robots_data = vrep_con.get_robots_data()
goal_data = vrep_con.get_goal_data()
obstacles_data = vrep_con.get_obstacles_data()

objects_msg.source = "vrep"
objects_msg.robots = prepare_robot_msg(robots_data)
objects_msg.goals = prepare_goal_msg(goal_data)
objects_msg.obstacles = prepare_obstacle_msg(obstacles_data)
vrep_data_publisher.publish(objects_msg)
vrep_con.finish_connection()
rospy.spin()
