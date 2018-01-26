"""
Utilities for traffic light module
"""

import numpy as np
from styx_msgs.msg import TrafficLightArray, TrafficLight
import rospy
import yaml


def get_given_traffic_lights():
    """
    Return given traffic light positions
    :return: TrafficLightArray
    """
    traffic_lights = TrafficLightArray()

    traffic_light_list = []

    tl_height = rospy.get_param("/tl_height")
    config_string = rospy.get_param("/traffic_light_config")
    traffic_light_positions = yaml.load(config_string)["light_positions"]

    for traffic_light_index, traffic_light_position in enumerate(traffic_light_positions):
        traffic_light = TrafficLight()

        traffic_light.pose.pose.position.x = traffic_light_position[0]
        traffic_light.pose.pose.position.y = traffic_light_position[1]
        traffic_light.pose.pose.position.z = tl_height
        traffic_light.state = TrafficLight.UNKNOWN
        traffic_light_list.append(traffic_light)

        traffic_lights.lights = traffic_light_list

    return traffic_lights


def get_waypoints_matrix(waypoints):
    """
    Converts waypoints listt to numpy matrix
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: 2D numpy array
    """

    waypoints_matrix = np.zeros(shape=(len(waypoints), 2), dtype=np.float32)

    for index, waypoint in enumerate(waypoints):
        waypoints_matrix[index, 0] = waypoint.pose.pose.position.x
        waypoints_matrix[index, 1] = waypoint.pose.pose.position.y

    return waypoints_matrix


def get_closest_waypoint_index(position, waypoints_matrix):
    """
    Given a pose and waypoints list, return index of waypoint closest to pose
    :param position: geometry_msgs.msgs.Position instance
    :param waypoints_matrix: numpy matrix with waypoints coordinates
    :return: integer index
    """

    x_distances = waypoints_matrix[:, 0] - position.x
    y_distances = waypoints_matrix[:, 1] - position.y

    squared_distances = x_distances ** 2 + y_distances ** 2
    return np.argmin(squared_distances)


def get_road_distance(waypoints):
    """
    Get road distance covered when following waypoints
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: float
    """

    total_distance = 0.0

    for index in range(1, len(waypoints)):

        x_distance = waypoints[index].pose.pose.position.x - waypoints[index - 1].pose.pose.position.x
        y_distance = waypoints[index].pose.pose.position.y - waypoints[index - 1].pose.pose.position.y

        distance = np.sqrt((x_distance**2) + (y_distance**2))

        total_distance += distance

    return total_distance


def get_closest_traffic_light_ahead_of_car(traffic_lights, car_position, waypoints):
    """
    Given list of traffic lights, car position and waypoints, return closest traffic light
    ahead of the car. This function wraps around the track, so that if car is at the end of the track,
    and closest traffic light is at track's beginning, it will be correctly reported
    :param traffic_lights: list of styx_msgs.msg.TrafficLight instances
    :param car_position: geometry_msgs.msgs.Pose instance
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: styx_msgs.msg.TrafficLight instance
    """

    waypoints_matrix = get_waypoints_matrix(waypoints)
    car_index = get_closest_waypoint_index(car_position, waypoints_matrix)

    # Arrange track waypoints so they start at car position
    waypoints_ahead = waypoints[car_index:] + waypoints[:car_index]
    waypoints_ahead_matrix = get_waypoints_matrix(waypoints_ahead)

    distances = []

    for traffic_light in traffic_lights:

        waypoint_index = get_closest_waypoint_index(traffic_light.pose.pose.position, waypoints_ahead_matrix)

        distance = get_road_distance(waypoints_ahead[:waypoint_index])
        distances.append(distance)

    closest_traffic_light_index = np.argmin(distances)

    return traffic_lights[closest_traffic_light_index]
