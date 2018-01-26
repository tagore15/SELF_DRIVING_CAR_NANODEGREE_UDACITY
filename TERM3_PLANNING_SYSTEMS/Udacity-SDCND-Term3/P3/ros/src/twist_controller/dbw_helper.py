"""
Utilities used by dbw_py
"""

import numpy as np
import rospy


def get_waypoints_coordinates_matrix(waypoints):
    """
    Given a list of waypoints, returns a numpy matrixes with x y coordinates
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: Nx2 numpy array where N is number of waypoints
    """

    points = []

    # Transform all points
    for waypoint in waypoints:

        x = waypoint.pose.pose.position.x
        y = waypoint.pose.pose.position.y

        points.append([x, y])

    return np.array(points)


def get_cross_track_error(waypoints, current_pose):
    """
    Given waypoints ahead of the car, fits polynomial to them, estimates expected y at current x pose and compares
    that to actual y to compute cross track error - a deviation from expected trajectory
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :param current_pose: geometry_msgs.msgs.Pose instance
    :return: float
    """

    origin = waypoints[0].pose.pose.position

    # Get waypoints into a matrix. shift them to origin
    waypoints_matrix = get_waypoints_coordinates_matrix(waypoints)
    shifted_waypoints_matrix = waypoints_matrix - np.array([origin.x, origin.y])

    # Get angle a waypoint a bit in the future makes with x-origin
    offset = 10
    angle = np.arctan2(shifted_waypoints_matrix[offset, 1], shifted_waypoints_matrix[offset, 0])

    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    # Rotated waypoints to origin, so they are mostly changing in positive x direction rather than y
    rotated_waypoints = np.dot(shifted_waypoints_matrix, rotation_matrix)

    # Fit a polynomial to waypoints
    degree = 2

    coefficients = np.polyfit(rotated_waypoints[:, 0], rotated_waypoints[:, 1], degree)

    shifted_current_point = np.array([current_pose.position.x - origin.x, current_pose.position.y - origin.y])
    rotated_current_point = np.dot(shifted_current_point, rotation_matrix)

    expected_value = np.polyval(coefficients, rotated_current_point[0])
    actual_value = rotated_current_point[1]

    return -(actual_value - expected_value)


