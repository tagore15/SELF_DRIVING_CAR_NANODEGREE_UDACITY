"""
Tests for tf_helper
"""
import styx_msgs.msg
import geometry_msgs.msg

import numpy as np

import src.tl_detector.tf_helper as tf_helper


def test_get_closest_traffic_light_ahead_of_car_first_light_closest():

    first_light = styx_msgs.msg.TrafficLight()
    first_light.pose.pose.position.x = 5
    first_light.pose.pose.position.y = 0
    first_light.pose.pose.position.z = 0

    second_light = styx_msgs.msg.TrafficLight()
    second_light.pose.pose.position.x = 7
    second_light.pose.pose.position.y = 0
    second_light.pose.pose.position.z = 0

    traffic_lights = [first_light, second_light]

    car_position = geometry_msgs.msg.Point()
    car_position.x = 2
    car_position.y = 0
    car_position.z = 0

    waypoints = []

    for x in range(0, 10):

        waypoint = styx_msgs.msg.Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = 0
        waypoint.pose.pose.position.z = 0

        waypoints.append(waypoint)

    expected = first_light
    actual = tf_helper.get_closest_traffic_light_ahead_of_car(traffic_lights, car_position, waypoints)

    assert expected == actual


def test_get_closest_traffic_light_ahead_of_car_second_light_closest():

    first_light = styx_msgs.msg.TrafficLight()
    first_light.pose.pose.position.x = 5
    first_light.pose.pose.position.y = 0
    first_light.pose.pose.position.z = 0

    second_light = styx_msgs.msg.TrafficLight()
    second_light.pose.pose.position.x = 3
    second_light.pose.pose.position.y = 0
    second_light.pose.pose.position.z = 0

    traffic_lights = [first_light, second_light]

    car_position = geometry_msgs.msg.Point()
    car_position.x = 2
    car_position.y = 0
    car_position.z = 0

    waypoints = []

    for x in range(0, 10):

        waypoint = styx_msgs.msg.Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = 0
        waypoint.pose.pose.position.z = 0

        waypoints.append(waypoint)

    expected = second_light
    actual = tf_helper.get_closest_traffic_light_ahead_of_car(traffic_lights, car_position, waypoints)

    assert expected == actual


def test_get_closest_traffic_light_ahead_of_car_car_at_end_of_track_closest_light_at_beginning_of_track():

    first_light = styx_msgs.msg.TrafficLight()
    first_light.pose.pose.position.x = 5
    first_light.pose.pose.position.y = 0
    first_light.pose.pose.position.z = 0

    second_light = styx_msgs.msg.TrafficLight()
    second_light.pose.pose.position.x = 7
    second_light.pose.pose.position.y = 0
    second_light.pose.pose.position.z = 0

    traffic_lights = [first_light, second_light]

    car_position = geometry_msgs.msg.Point()
    car_position.x = 8
    car_position.y = 0
    car_position.z = 0

    waypoints = []

    for x in range(0, 10):

        waypoint = styx_msgs.msg.Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = 0
        waypoint.pose.pose.position.z = 0

        waypoints.append(waypoint)

    expected = first_light
    actual = tf_helper.get_closest_traffic_light_ahead_of_car(traffic_lights, car_position, waypoints)

    assert expected == actual
