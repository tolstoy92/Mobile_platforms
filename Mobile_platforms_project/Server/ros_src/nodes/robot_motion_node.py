#!/usr/bin/env python

import rospy
import vrep_communicator.vrep_constants as const
from platforms_server.msg import RobotData, WheelRotationData
from vision.Fileds_objects import Point
from math import fabs, sqrt, acos, pi
from time import sleep

def get_angle_difference(robot_dir, desired_dir):
    angle_difference = desired_dir - robot_dir
    if angle_difference > 180:
        angle_difference = -(360 - angle_difference)
    elif angle_difference < -180:
        angle_difference = 360 + angle_difference
    return angle_difference

def get_PID_impact(old_error, error_sum, error):
    velocity = const.MOVEMENT_SPEED
    error_sum = error_sum + error
    if error_sum < const.iMin:
        error_sum = const.iMin
    elif error_sum > const.iMax:
        error_sum = const.iMax
    up = const.kp * error
    ui = const.ki * error_sum
    ud = const.kd * (error - old_error)
    old_error = error
    u = up + ui + ud
    if u > 0:
        left_u = velocity - fabs(u)
        right_u = velocity
    else:
        left_u = velocity
        right_u = velocity - fabs(u)
    return left_u, right_u, old_error, error_sum

def movement(robot_data):
    msg = WheelRotationData()
    msg.id = robot_data.id
    if not robot_data.on_finish:
        start = robot_data.actual_point.start
        finish = robot_data.actual_point.finish
        if robot_data.actual_point.start == robot_data.actual_point.finish:
            sleep(robot_data.actual_point.duration)

            msg.left_velocity = 0
            msg.right_velocity = 0
            msg.rotation = True
            msg.goal_reached = True
        else:
            msg.goal_reached = False
            robot_pos = Point((robot_data.position.x, robot_data.position.y))
            goal = robot_data.actual_point.finish
            goal_pos = Point((goal.x, goal.y))
            robot_dir = robot_data.direction
            desired_dir = robot_data.angle_to_actual_point
            angle_difference = get_angle_difference(robot_dir, desired_dir)
            if robot_data.rotation:
                if fabs(angle_difference) > const.ANGLE_ERROR:
                    if angle_difference > 0:
                        msg.left_velocity = -const.ROTATION_SPEED
                        msg.right_velocity = const.ROTATION_SPEED
                    else:
                        msg.left_velocity = const.ROTATION_SPEED
                        msg.right_velocity = -const.ROTATION_SPEED
                    msg.rotation = True
                else:
                    msg.left_velocity = 0
                    msg.right_velocity = 0
                    msg.rotation = False
            else:
                global old_error
                global error_sum
                if robot_pos.get_distance(goal_pos) > const.DISTANCE_ERROR:
                    left_velo, right_velo, old_error, error_sum = get_PID_impact(old_error, \
                                                                                 error_sum, angle_difference)
                    msg.left_velocity = left_velo
                    msg.right_velocity = right_velo
                    msg.rotation = False
                else:
                    msg.left_velocity = 0
                    msg.right_velocity = 0
                    msg.rotation = True
                    msg.goal_reached = True
        wheel_rotation_pub.publish(msg)
    else:
        stop_func()


def stop_func():
    global robot_data_sub
    robot_data_sub.unregister()

old_error = 0
error_sum = 0
rospy.init_node("robot_motion_node")
robot_data_sub = rospy.Subscriber("robot_movement_data", RobotData, movement)
wheel_rotation_pub = rospy.Publisher("wheel_rotation_data", WheelRotationData)

rospy.on_shutdown(stop_func)
rospy.spin()