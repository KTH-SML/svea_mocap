#!/usr/bin/env python

"""
Module containing localization interface for motion capture
"""

from __future__ import division
from threading import Thread, Event
import rospy
import tf
import math
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from svea.states import VehicleState

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se "
__status__ = "Development"


class MotionCaptureInterface(object):
    """Interface handling the reception of state information from the
    motion capture system. This object can take on several callback
    functions and execute them as soon as state information is
    available.

    :param mocap_name: Name of mocap model in Qualisys software;
                                The name will be effectively be added as a
                                namespace to the topics used by the
                                corresponding localization node i.e
                                `qualisys/model_name/odom`, defaults to
                                ''
    :type mocap_name: str, optional
    """

    def __init__(self, mocap_name=''):
        self.model_name = mocap_name
        self._odom_sub = None
        self._vel_sub = None

        self._curr_vel_twist = None
        self.state = VehicleState()
        self.last_time = float('nan')

        self._x_offset = 0.0 # [m]
        self._y_offset = 0.0

        self.is_ready = False
        self.tf_listener = tf.TransformListener()

        self._ready_event = Event()
        rospy.on_shutdown(self._shutdown_callback)

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def update_name(self, name):
        self.model_name = name
        self._odom_topic = 'qualisys/' + self.model_name + '/odom'
        self._vel_topic = 'qualisys/' + self.model_name + '/velocity'
        # check if old subs need to be removed
        if not self._odom_sub is None:
            self._odom_sub.unregister()
        if not self._vel_sub is None:
            self._vel_sub.unregister()
        self._start_listen()

    def set_model_offset(self, x, y):
        self._x_offset = x
        self._y_offset = y

    def start(self):
        """Spins up ROS background thread; must be called to start
        receiving data

        :return: itself
        :rtype: MotionCaptureInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _wait_until_ready(self, timeout=20.0):
        tic = rospy.get_time()
        self._ready_event.wait(timeout)
        toc = rospy.get_time()
        wait = toc - tic
        return wait < timeout

    def _shutdown_callback(self):
        self._ready_event.set()

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Motion Capture Interface Node for "
                      + self.model_name)
        self.node_name = 'motion_capture_node'
        self.update_name(self.model_name)
        self.is_ready = self._wait_until_ready()
        if not self.is_ready:
            rospy.logwarn("Motion Capture not responding during start of "
                          "Motion Caputer. Setting ready anyway.")
        self.is_ready = True
        rospy.loginfo("{} Motion Capture Interface successfully initialized"
                      .format(self.model_name))

        rospy.spin()

    def _start_listen(self):
        self._odom_sub = rospy.Subscriber(self._odom_topic,
                                           Odometry,
                                           self._read_odom_msg,
                                           tcp_nodelay=True,
                                           queue_size=1)
        self._vel_sub = rospy.Subscriber(self._vel_topic,
                                        TwistStamped,
                                        self._read_vel_msg,
                                        tcp_nodelay=True,
                                        queue_size=1)

    def fix_twist(self, odom_msg):
        odom_msg.twist.twist = self._curr_vel_twist
        return odom_msg

    def _read_odom_msg(self, msg):
        if not self._curr_vel_twist is None:
            msg = self.fix_twist(msg)
            #self.state.odometry_msg = msg     # compromises reference transform if uncommented

            # Apply the model offsets (if any)
            x = msg.pose.pose.position.x + self._x_offset
            y = msg.pose.pose.position.y + self._y_offset
            yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

            # Transform coordinates from mocap to map frame
            x, y, yaw = self.transform_to_map_frame(x, y, yaw)

            linear_x = msg.twist.twist.linear.x  # Velocity in the x direction of the mocap frame
            linear_y = msg.twist.twist.linear.y  # Velocity in the y direction of the mocap frame
            v = linear_x * math.cos(yaw+math.pi/2) + linear_y * math.sin(yaw+math.pi/2)

            # Update the state
            self.state.x = x
            self.state.y = y
            self.state.yaw = yaw
            self.state.v = v
            self.state.frame_id = "map"
            self.state.time_stamp = rospy.Time.now()

            self.last_time = rospy.get_time()
            self._ready_event.set()
            self._ready_event.clear()

            for cb in self.callbacks:
                cb(self.state)
                
    def get_yaw_from_quaternion(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def _read_vel_msg(self, msg):
        self._curr_vel_twist = msg.twist

    def add_callback(self, cb):
        """Add state callback. Every function passed into this method
        will be called whenever new state information comes in from the
        motion capture system.

        :param cb: A callback function intended for responding to the
                   reception of state info
        :type cb: function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))

    def transform_to_map_frame(self, x, y, yaw):
        """Transforms coordinates from mocap to map frame using tf."""
        pose = PoseStamped()
        pose.header.frame_id = "mocap"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        try:
            self.tf_listener.waitForTransform("map", "mocap", rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = self.tf_listener.transformPose("map", pose)
            # Extract transformed coordinates
            x_transformed = transformed_pose.pose.position.x
            y_transformed = transformed_pose.pose.position.y
            _, _, yaw_transformed = tf.transformations.euler_from_quaternion([
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w,
            ])

            return x_transformed, y_transformed, yaw_transformed
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Could not transform coordinates to map frame: %s", e)
            return x, y, yaw  # Return original coordinates if transform fails