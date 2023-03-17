#! /usr/bin/env python3
import roslib

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


if __name__ == '__main__':
    # Init node
	rospy.init_node('tf_mocap_to_map')
    # Load parameters
	x = load_param('~x', 0.0)
	y = load_param('~y', 0.0)
	z = load_param('~z', 0.0)
	yaw = load_param('~yaw', 0.0)
	pitch = load_param('~pitch', 0.0)
	roll = load_param('~roll', 0.0)
	parent_frame = load_param('~frame_id', 'mocap')
	child_frame = load_param('~child_frame', 'map')
     
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	static_transformStamped = geometry_msgs.msg.TransformStamped()

	static_transformStamped.header.stamp = rospy.Time.now()
	static_transformStamped.header.frame_id = parent_frame
	static_transformStamped.child_frame_id = child_frame
	static_transformStamped.transform.translation.x = float(x)
	static_transformStamped.transform.translation.y = float(y)
	static_transformStamped.transform.translation.z = float(z)
	quat = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))
	static_transformStamped.transform.rotation.x = quat[0]
	static_transformStamped.transform.rotation.y = quat[1]
	static_transformStamped.transform.rotation.z = quat[2]
	static_transformStamped.transform.rotation.w = quat[3]

	broadcaster.sendTransform(static_transformStamped)
	rospy.spin()