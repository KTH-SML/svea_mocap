#!/usr/bin/env python

"""
Module containing localization interface for motion capture
"""

from __future__ import division
from threading import Thread, Event
import rospy
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
        self._mocap_topic = 'qualisys/' + self.model_name + '/odom'
        self._mocap_sub = None

        self.state = VehicleState()
        self.last_time = float('nan')

        self.is_ready = False
        self._ready_event = Event()
        rospy.on_shutdown(self._shutdown_callback)

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def update_name(self, name):
        self.model_name = name
        self._mocap_topic = 'qualisys/' + self.model_name + '/odom'
        if not self._mocap_sub is None:
            self._mocap_sub.unregister()
        self._mocap_sub = rospy.Subscriber(self._mocap_topic,
                                        Odometry,
                                        self._read_mocap_msg,
                                        tcp_nodelay=True,
                                        queue_size=1)

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
        self._start_listen()
        self.is_ready = self._wait_until_ready()
        if not self.is_ready:
            rospy.logwarn("Motion Capture not responding during start of "
                          "Motion Caputer. Setting ready anyway.")
        self.is_ready = True
        rospy.loginfo("{} Motion Capture Interface successfully initialized"
                      .format(self.model_name))

        rospy.spin()

    def _start_listen(self):
        self._mocap_sub = rospy.Subscriber(self._mocap_topic,
                                           Odometry,
                                           self._read_mocap_msg,
                                           tcp_nodelay=True,
                                           queue_size=1)

    def _read_mocap_msg(self, msg):
        self.state.odometry_msg = msg
        self.last_time = rospy.get_time()
        self._ready_event.set()
        self._ready_event.clear()

        for cb in self.callbacks:
            cb(self.state)

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
