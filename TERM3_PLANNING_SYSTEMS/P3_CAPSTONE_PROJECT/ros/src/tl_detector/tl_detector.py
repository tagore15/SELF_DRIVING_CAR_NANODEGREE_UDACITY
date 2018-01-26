#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifierCV, TLClassifier
import tf
import tf.transformations
import cv2
import tf_helper
import numpy as np
import yaml
import geometry_msgs.msg


STATE_COUNT_THRESHOLD = 3


class TLDetector(object):

    def __init__(self):
        rospy.init_node('tl_detector')

        self.car_pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.traffic_positions = tf_helper.get_given_traffic_lights()

        self.last_traffic_light_state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.last_reported_traffic_light_id = None
        self.last_reported_traffic_light_time = None

        self.traffic_lights = None
        self.image = None

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and 
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()

        self.experiment_environment = rospy.get_param('/experiment_environment', "site")
        self.light_classifier = TLClassifier(self.experiment_environment)
        # self.light_classifier = TLClassifierCV()

        self.listener = tf.TransformListener()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        self.upcoming_stop_light_pub = rospy.Publisher(
            '/upcoming_stop_light_position', geometry_msgs.msg.Point, queue_size=1)

        self.image_pub = rospy.Publisher('/camera/my_image', Image, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):

        self.car_pose = msg.pose

        # For debugging(Ground Truth data)
        # arguments = [self.traffic_lights, self.car_pose, self.waypoints, self.image]

        arguments = [self.traffic_positions, self.car_pose, self.waypoints, self.image]
        are_arguments_available = all([x is not None for x in arguments])

        if are_arguments_available:

            # Get closest traffic light
            traffic_light = tf_helper.get_closest_traffic_light_ahead_of_car(
                self.traffic_positions.lights, self.car_pose.position, self.waypoints)

            # These values seem so be wrong - Udacity keeps on putting in config different values that what camera
            # actually publishes.
            # image_width = self.config["camera_info"]["image_width"]
            # image_height = self.config["camera_info"]["image_height"]

            # Therefore simply check image size
            self.camera_image = self.image
            self.camera_image.encoding = "rgb8"
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            image_height = cv_image.shape[0]
            image_width = cv_image.shape[1]

            x, y = self.project_to_image_plane(
                traffic_light.pose.pose.position, self.car_pose, image_width, image_height)

            simulator_traffic_light_in_view = 0 < x < image_width and 0 < y < image_height
            # As of this writing, site camera mapping is broken (thanks, Udacity...), so we will just process all
            # images on site
            site_traffic_light_in_view = True

            traffic_light_in_view = simulator_traffic_light_in_view if self.experiment_environment == "simulator" \
                else site_traffic_light_in_view

            # Only try to classify image if traffic light is within it
            if traffic_light_in_view:

                traffic_light_state = self.light_classifier.get_classification(cv_image)

                # lights_map = {0: "Red", 1: "Yellow", 2: "Green"}
                # rospy.logwarn("Detected light: {}".format(lights_map.get(traffic_light_state, "Other")))

                cv2.circle(cv_image, (x, y), radius=50, color=(255, 0, 0), thickness=12)
                marked_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                self.image_pub.publish(marked_image)

                if traffic_light_state == TrafficLight.RED or traffic_light == TrafficLight.YELLOW:
                    self.upcoming_stop_light_pub.publish(traffic_light.pose.pose.position)

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.traffic_lights = msg.lights

    def image_cb(self, msg):
        self.image = msg

    def project_to_image_plane(self, point_in_world, car_pose, image_width, image_height):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world
            car_pose: current car pose
            image_width: camera image width
            image_height: camera image height

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config["camera_info"]["focal_length_x"]
        fy = self.config["camera_info"]["focal_length_y"]

        # light = None
        # light_positions = self.config['light_positions']
        # if(self.pose):
        #     car_position = self.get_closest_waypoint(self.pose.pose)

        # Commenting out trans and rot code - we aren't using them for now, as they seem broken and
        # are received using blocking code - thus somewhat slow
        # # get transform between pose of camera and world frame
        # trans = None
        # try:
        #     now = rospy.Time.now()
        #     self.listener.waitForTransform("/base_link",
        #                                    "/world", now, rospy.Duration(1.0))
        #     (trans, rot) = self.listener.lookupTransform("/base_link",
        #                                                  "/world", now)
        #
        # except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
        #     rospy.logerr("Failed to find camera to map transform")

        #
        # # rospy.logwarn("Transform shift is: {}".format(trans))
        # # rospy.logwarn("Rotation is: {}".format(rot))

        # TODO Use transform and rotation to calculate 2D position of light in image
        world_coordinates_point = np.array(
            [point_in_world.x, point_in_world.y, point_in_world.z], dtype=np.float32).reshape(3, 1)

        car_position = np.array([car_pose.position.x, car_pose.position.y, car_pose.position.z],
                                dtype=np.float32).reshape(3, 1)
        camera_offset = np.array([1.0, 0, 1.2], dtype=np.float32).reshape(3, 1)
        # translation_vector = np.array(trans, dtype=np.float32).reshape(3, 1)
        translation_vector = car_position + camera_offset

        # Move point to camera origin
        world_coordinates_point_shifted_to_camera_coordinates = world_coordinates_point - translation_vector

        homogenous_vector = np.ones(shape=(4, 1), dtype=np.float32)
        homogenous_vector[:3] = world_coordinates_point_shifted_to_camera_coordinates

        quaternion = np.array([
            car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w],
            dtype=np.float32)

        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
        rotation_matrix = tf.transformations.euler_matrix(*euler_angles)

        point_in_camera_coordinates = np.dot(rotation_matrix, homogenous_vector)

        x = (fx * point_in_camera_coordinates[0] * point_in_camera_coordinates[2]) + (image_width / 2)
        y = (fy * point_in_camera_coordinates[1] * point_in_camera_coordinates[2]) + (image_height / 2)

        return int(x), int(y)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
