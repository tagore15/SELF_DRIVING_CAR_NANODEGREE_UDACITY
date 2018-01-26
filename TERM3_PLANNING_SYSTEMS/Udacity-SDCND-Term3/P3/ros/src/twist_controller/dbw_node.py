#!/usr/bin/env python

import math

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import geometry_msgs.msg
import styx_msgs.msg
import std_msgs.msg

from twist_controller import Controller
import dbw_helper
import pid


'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


class DBWNode(object):

    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.is_drive_by_wire_enable = False
        self.last_twist_command = None
        self.current_velocity = None
        self.current_pose = None
        self.final_waypoints = None
        self.previous_loop_time = rospy.get_rostime()
        self.previous_debug_time = rospy.get_rostime()

        self.throttle_pid = pid.PID(kp=0.5, ki=0.05, kd=0.0, mn=decel_limit, mx=0.5 * accel_limit)
        self.brake_pid = pid.PID(kp=125.0, ki=0.0, kd=3.0, mn=brake_deadband, mx=5000)
        self.steering_pid = pid.PID(kp=0.5, ki=0.0, kd=1.0, mn=-max_steer_angle, mx=max_steer_angle)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(self.throttle_pid, self.brake_pid, self.steering_pid)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_commands_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.drive_by_wire_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/current_pose', geometry_msgs.msg.PoseStamped, self.current_pose_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', styx_msgs.msg.Lane, self.final_waypoints_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)  # Frequency
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            data = [self.last_twist_command, self.current_velocity, self.current_pose, self.final_waypoints]
            is_all_data_availabe = all([x is not None for x in data])

            if self.is_drive_by_wire_enable and is_all_data_availabe:

                current_time = rospy.get_rostime()
                ros_duration = current_time - self.previous_loop_time
                duration_in_seconds = ros_duration.secs + (1e-9 * ros_duration.nsecs)
                self.previous_loop_time = current_time

                # Base linear velocity error on difference between current speed and desired speed x waypoints later
                linear_velocity_error = self.final_waypoints[1].twist.twist.linear.x - self.current_velocity.linear.x
                cross_track_error = dbw_helper.get_cross_track_error(self.final_waypoints, self.current_pose)

                # Primitive command
                throttle, brake, steering = self.controller.control(
                    linear_velocity_error, cross_track_error, duration_in_seconds)

                self.publish(throttle, brake, steering)
                # self.print_debug_info(throttle, brake, steering, linear_velocity_error)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_commands_cb(self, msg):

        self.last_twist_command = msg.twist

    def drive_by_wire_enabled_cb(self, msg):

        self.is_drive_by_wire_enable = bool(msg.data)

        if self.is_drive_by_wire_enable is True:

            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.steering_pid.reset()

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

    def print_debug_info(self, throttle, brake, steering, linear_velocity_error):
        """
        Print debugging commands. Only prints out if enough time has passed since last printout
        """

        current_time = rospy.get_rostime()
        ros_duration_since_debug = current_time - self.previous_debug_time
        duration_since_debug_in_seconds = ros_duration_since_debug.secs + (1e-9 * ros_duration_since_debug.nsecs)

        if duration_since_debug_in_seconds > 0.5:

            rospy.logwarn("linear_velocity_error: {}".format(linear_velocity_error))
            rospy.logwarn("Throttle command: {}".format(throttle))
            rospy.logwarn("Brake command: {}".format(brake))

            self.previous_debug_time = current_time


if __name__ == '__main__':
    DBWNode()
