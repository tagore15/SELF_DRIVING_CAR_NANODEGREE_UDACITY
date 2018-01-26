"""
Tests for dbw helper
"""

import styx_msgs.msg
import geometry_msgs.msg
import numpy as np

import src.twist_controller.dbw_helper as dbw_helper


def test_get_waypoints_coordinates_matrix():

    first_waypoint = styx_msgs.msg.Waypoint()
    first_waypoint.pose.pose.position.x = 0
    first_waypoint.pose.pose.position.y = 2

    second_waypoint = styx_msgs.msg.Waypoint()
    second_waypoint.pose.pose.position.x = 8
    second_waypoint.pose.pose.position.y = 20

    third_waypoint = styx_msgs.msg.Waypoint()
    third_waypoint.pose.pose.position.x = 50
    third_waypoint.pose.pose.position.y = -4

    waypoints = [first_waypoint, second_waypoint, third_waypoint]

    expected = np.array([
        [0, 2],
        [8, 20],
        [50, -4]
    ])

    actual = dbw_helper.get_waypoints_coordinates_matrix(waypoints)

    assert np.all(expected == actual)