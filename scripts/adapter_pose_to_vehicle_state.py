#! /usr/bin/env python3

import rospy
import math
from svea_msgs.msg import VehicleState as VehicleStateMsg
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_matrix, euler_from_matrix, euler_matrix
import tf
import numpy as np

__license__ = "MIT"
__maintainer__ = "Michele Pestarino, Federico Sacco"
__email__ = "micpes@kth.se, fsacco@ug.kth.se"
__status__ = "Development"

__all__ = [
    'AdapterPoseToVehicleState',
]

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class AdapterPoseToVehicleState:
    """
    Class needed to publish on the vehicle's state topic data coming
    from mocap (considered as ground truth)
    """

    def __init__(self, vehicle_name: str = ''):
        """
        Init method for class AdapterPoseToVehicleState

        :param vehicle_name: vehicle name, defaults to ''
        :type vehicle_name: str, optional
        """
        # Offset angle between the mocap frame (of the real world) and the map frame
        self.OFFSET_ANGLE = -math.pi/2
        self.T_MATRIX_4 = euler_matrix(0, 0, self.OFFSET_ANGLE)
        # Create rotation matrix given the offset angle and linear misalignment between mocap and map frames
        LINEAR_PART_MOCAP2MAP =  np.transpose(np.array([5.619764999999999, 5.870124000000001, 0]))
        self.T_MATRIX_4[0:3,3] = LINEAR_PART_MOCAP2MAP
        
        self.vehicle_R_mocap = []

        # Get vehicle name from parameters
        self.vehicle_name = vehicle_name
        # Set svea on board localization topic's name
        self._mocap_pose_topic = load_param('~input_topic', f'/qualisys/{vehicle_name}/pose')
        # Set mocap topic's name
        self._svea_state_topic = load_param('~output_topic', '/state')
        # Mocap's velocity topic name
        self._mocap_vel_topic = load_param('~velocity_topic', f'/qualisys/{vehicle_name}/velocity')
        self._verbose = load_param('~state_verbose', True)
        # Initi current velocity twist to [0; 0] (to avoid having a first null reading from topic)
        self._curr_vel_twist = np.array([0, 0])

        # Define publisher for VehicleState topic
        self._state_pub = None
        # Define subscriber for mocap pose topic
        self._pose_sub = None
        # Define subscriber for mocap velocity topic
        self._vel_sub = None 
        # Initialize state message
        self._state_msg = VehicleStateMsg()

    def init_ros_interface(self):
        """
        Function defining publishers/subscribers for the adapter_pose_to_vehicle_state node
        """
        # Define mocap pose subscriber
        self._pose_sub = rospy.Subscriber(self._mocap_pose_topic,
                         PoseStamped,
                         self._pose_callback,
                         queue_size=1)
        
        # Define vehicle state publisher 
        self._state_pub = rospy.Publisher(self._svea_state_topic, VehicleStateMsg, queue_size=1)

        self._vel_sub = rospy.Subscriber(self._mocap_vel_topic,
                                        TwistStamped,
                                        self._read_vel_msg,
                                        tcp_nodelay=True,
                                        queue_size=1)

    def _pose_callback(self, msg):
        """
        Callback for the mocap pose subscriber
        
        :param msg: msg from the mocap pose topic 
        :type msg: PoseStamped
        """
        # Correct pose based on the angle offset between map and mocap frames
        x_corr, y_corr, yaw_corr = self._correct_mocap_coordinates(msg.pose.position.x, msg.pose.position.y, msg.pose.orientation)
        # Assign vehicle state message fields
        self._state_msg.header = msg.header
        self._state_msg.header.frame_id = 'map'
        self._state_msg.child_frame_id = 'base_link'
        self._state_msg.x = x_corr
        self._state_msg.y = y_corr
        self._state_msg.yaw = yaw_corr
        # Compute velocity given current vehicle's twist wrt mocap frame and current quaternion of the vehicle wrt mocap frame
        self._state_msg.v = self._compute_vehicle_velocity(msg.pose.orientation)
        # Set covariances to 0
        # TODO: get covariances from mocap odom topic
        self._state_msg.covariance = [0] * 16

        if self._verbose:
            print("x = " + str(self._state_msg.x))
            print("y = " + str(self._state_msg.y))
            print("yaw = " + str(self._state_msg.yaw))
            print("v = " + str(self._state_msg.v) + "\n")

        # Publish state message
        self._state_pub.publish(self._state_msg)

    def _read_vel_msg(self, msg):
        # Save current twist wrt mocap frame of x and y axis
        self._curr_vel_twist = np.array([msg.twist.linear.x, msg.twist.linear.y])

    def _compute_vehicle_velocity(self, quaternion):
        """
        Method used to compute the vehicle's velocity given the mocap's twist
        
        :param quaternion: quaternion used to extract and correct velocity
        :type quaternion: Quaternion

        :return: v[0] vehicle velocity
        :rtype: float
        """
        # Get svea's rotation matrix from pose quaternion wrt mocap frame
        vehicle_R_mocap = quaternion_matrix([quaternion.x, 
                                                  quaternion.y, 
                                                  quaternion.z, 
                                                  quaternion.w])
        # Get vehicle yaw wrt mocap frame
        (_, _, vehicle_yaw_mocap) = euler_from_matrix(vehicle_R_mocap)
        # Compute vehicle velocity by reprojecting the twist of the vehicle wrt mocap frame, onto the vehicle frame
        # itself, then extract the x component (which is the vehicle velocity)
        self.vehicle_R_mocap = [[math.cos(-vehicle_yaw_mocap), -math.sin(-vehicle_yaw_mocap)],
                                [math.sin(-vehicle_yaw_mocap), math.cos(-vehicle_yaw_mocap)]]
        v = np.matmul(self.vehicle_R_mocap, np.transpose(np.array(self._curr_vel_twist)))
        return v[0]
        

    def _correct_mocap_coordinates(self, x, y, quaternion):
        """
        Method used to correct the mocap pose (if some misalignment between its frame and the map frame is present)
        
        :param x: x coordinate to be corrected 
        :type x: float
        :param y: y coordinate to be corrected 
        :type y: float
        :param quaternion: quaternion used to extract and correct yaw angle 
        :type quaternion: Quaternion

        :return: rotate_point[0] corrected x coordinate
        :rtype: float
        :return: rotate_point[1] corrected y coordinate
        :rtype: float
        :return: mocap_yaw corrected yaw angle
        :rtype: float
        """
        # Get svea's rotation matrix from pose quaternion
        svea_T_mocap = quaternion_matrix([quaternion.x, 
                                                  quaternion.y, 
                                                  quaternion.z, 
                                                  quaternion.w])
        # Add translational part to transofmration matrix
        svea_T_mocap[0:3,3] = np.transpose(np.array([x,y,0]))

        # Apply 4 dimension square rotation matrix (rotate svea's yaw)
        svea_T_map = np.matmul(self.T_MATRIX_4, svea_T_mocap)

        # Get correct yaw (from manipulated rotation matrix)
        (_, _, mocap_yaw) = euler_from_matrix(svea_T_map)
        return svea_T_map[0,3], svea_T_map[1,3], mocap_yaw
        
        
if __name__ == '__main__':
    ## Start node ##
    rospy.init_node('adapter_pose_to_vehicle_state')
    # Instantiate object of class AdapterPoseToVehicleState implementing the node
    adapter_node = AdapterPoseToVehicleState(vehicle_name='svea7')
    # Init node and start listeners
    adapter_node.init_ros_interface()
    
    # Spin node 
    rospy.spin()
