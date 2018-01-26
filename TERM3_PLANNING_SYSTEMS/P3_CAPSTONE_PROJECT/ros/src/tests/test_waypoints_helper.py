"""
Tests for waypoints helper
"""

import styx_msgs.msg
import geometry_msgs.msg
import numpy as np

import src.waypoint_updater.waypoints_helper as waypoints_helper


def test_get_closest_waypoint_index():

    pose = geometry_msgs.msg.Pose()
    pose.position.x = 10
    pose.position.y = 10

    waypoints_matrix = np.array([
        [0, 0],
        [8, 8],
        [20, 20]
    ])

    assert 1 == waypoints_helper.get_closest_waypoint_index(pose.position, waypoints_matrix)


def test_get_sublist_simple():

    elements = [1, 2, 3, 4, 5]
    start_index = 1
    size = 2

    expected = [2, 3]
    actual = waypoints_helper.get_sublist(elements, start_index, size)

    assert expected == actual


def test_get_sublist_wrapped():

    elements = [1, 2, 3, 4, 5]
    start_index = 3
    size = 3

    expected = [4, 5, 1]
    actual = waypoints_helper.get_sublist(elements, start_index, size)

    assert expected == actual


def test_get_road_distance_90_deg_turn():

    first = styx_msgs.msg.Waypoint()
    first.pose.pose.position.x = 0
    first.pose.pose.position.y = 0

    second = styx_msgs.msg.Waypoint()
    second.pose.pose.position.x = 10
    second.pose.pose.position.y = 0

    third = styx_msgs.msg.Waypoint()
    third.pose.pose.position.x = 10
    third.pose.pose.position.y = 10

    waypoints = [first, second, third]

    expected = 20
    actual = waypoints_helper.get_road_distance(waypoints)

    assert np.isclose(expected, actual)


def test_get_road_distance_left_and_right_turn():
    first = styx_msgs.msg.Waypoint()
    first.pose.pose.position.x = 0
    first.pose.pose.position.y = 0

    second = styx_msgs.msg.Waypoint()
    second.pose.pose.position.x = 10
    second.pose.pose.position.y = 10

    third = styx_msgs.msg.Waypoint()
    third.pose.pose.position.x = 20
    third.pose.pose.position.y = 0

    waypoints = [first, second, third]

    expected = 2.0 * np.sqrt(200)
    actual = waypoints_helper.get_road_distance(waypoints)

    assert np.isclose(expected, actual)


def test_get_road_distance_not_symmetrical_90_deg_turn():
    first = styx_msgs.msg.Waypoint()
    first.pose.pose.position.x = 0
    first.pose.pose.position.y = 0

    second = styx_msgs.msg.Waypoint()
    second.pose.pose.position.x = 10
    second.pose.pose.position.y = 0

    third = styx_msgs.msg.Waypoint()
    third.pose.pose.position.x = 10
    third.pose.pose.position.y = 5

    waypoints = [first, second, third]

    expected = 15.0
    actual = waypoints_helper.get_road_distance(waypoints)

    assert np.isclose(expected, actual)


def test_get_road_distance_not_symmetrical_left_and_right_turns():
    first = styx_msgs.msg.Waypoint()
    first.pose.pose.position.x = 0
    first.pose.pose.position.y = 0

    second = styx_msgs.msg.Waypoint()
    second.pose.pose.position.x = 10
    second.pose.pose.position.y = 20

    third = styx_msgs.msg.Waypoint()
    third.pose.pose.position.x = 5
    third.pose.pose.position.y = -10

    waypoints = [first, second, third]

    expected = np.sqrt(500) + np.sqrt(925)
    actual = waypoints_helper.get_road_distance(waypoints)

    assert np.isclose(expected, actual)


def test_is_traffic_light_ahead_of_car_light_is_ahead_of_car():

    waypoints = []

    for x in range(0, 10):

        waypoint = styx_msgs.msg.Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = 0
        waypoint.pose.pose.position.z = 0

        waypoints.append(waypoint)

    car_position = geometry_msgs.msg.Point()
    car_position.x = 2
    car_position.y = 0
    car_position.z = 0

    light_position = geometry_msgs.msg.Point()
    light_position.x = 5
    light_position.y = 0
    light_position.z = 0

    expected = True
    actual = waypoints_helper.is_traffic_light_ahead_of_car(waypoints, car_position, light_position)

    assert expected == actual


def test_is_traffic_light_ahead_of_car_light_is_behind_the_car():

    waypoints = []

    for x in range(0, 10):

        waypoint = styx_msgs.msg.Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = 0
        waypoint.pose.pose.position.z = 0

        waypoints.append(waypoint)

    car_position = geometry_msgs.msg.Point()
    car_position.x = 5
    car_position.y = 0
    car_position.z = 0

    light_position = geometry_msgs.msg.Point()
    light_position.x = 3
    light_position.y = 0
    light_position.z = 0

    expected = False
    actual = waypoints_helper.is_traffic_light_ahead_of_car(waypoints, car_position, light_position)

    assert expected == actual


def test_is_traffic_light_ahead_of_car_light_is_ahead_of_waypoints():

    waypoints = []

    for x in range(0, 10):

        waypoint = styx_msgs.msg.Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = 0
        waypoint.pose.pose.position.z = 0

        waypoints.append(waypoint)

    car_position = geometry_msgs.msg.Point()
    car_position.x = 5
    car_position.y = 0
    car_position.z = 0

    light_position = geometry_msgs.msg.Point()
    light_position.x = 20
    light_position.y = 0
    light_position.z = 0

    expected = False
    actual = waypoints_helper.is_traffic_light_ahead_of_car(waypoints, car_position, light_position)

    assert expected == actual
