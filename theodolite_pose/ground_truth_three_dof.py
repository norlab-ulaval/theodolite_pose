import time
import numpy as np
import rclpy
from rclpy.node import Node
from theodolite_node_msgs.msg import TheodoliteCoordsStamped
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_action_status_default

PRISM_CONSTANT = 0.01

class GroundTruth(Node):
    def __init__(self):
        super().__init__('ground_truth_subscriber')
        input_topic = self.declare_parameter('input_topic', '/theodolite_master/theodolite_data').value
        output_topic = self.declare_parameter('output_topic', '/theodolite_master/theodolite_pose').value
        self.create_subscription(
            TheodoliteCoordsStamped,
            input_topic,
            self.listener_callback,
            10)
        self.tf_sub = self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_sub = self.create_subscription(
            TFMessage, 
            "/tf_static", 
            self.tf_callback, 
            qos_profile_action_status_default
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            output_topic,
            10)
        self.prism_id = -1
        self.prism_positions = []
        self.current_sums = np.zeros(3)
        self.nb_poses = 0
        self.Q1, self.Q2, self.Q3 = None, None, None
        self.T_theodo_to_map = None
        self.T_odom_to_base_link = None
        self.T_map_to_odom = None
        self.tf_acquired = False
        self.calibration_computed = False
        self.timer = self.create_timer(10, self.timer_callback)
        self.timer.cancel()

    def listener_callback(self, msg):
        self.pose = PoseStamped()
        if msg.status == 0:
            self.pose.header.frame_id = 'theodolite'
            self.pose.header.stamp.sec = msg.header.stamp.sec
            self.pose.header.stamp.nanosec = msg.header.stamp.nanosec
            distance = msg.distance + PRISM_CONSTANT
            x = distance * np.cos(np.pi/2 - msg.azimuth) * np.sin(msg.elevation)
            y = distance * np.sin(np.pi/2 - msg.azimuth) * np.sin(msg.elevation)
            z = distance * np.cos(msg.elevation)
            position = np.array([x, y, z])
            if self.calibration_computed:
                self.publish_pose(position)
            elif self.prism_id >= 3 and self.euclidian_distance(position, self.prism_positions[0]) < 0.1:
                self.get_logger().info("Back to prism1, computing calibration...")
                self.compute_calibration()
                self.calibration_computed = True
                self.get_logger().info("Calibration computed!")
            else:
                self.update_calibration(position)

    def tf_callback(self, tf_msg):
        for tf in tf_msg.transforms:
            if tf.child_frame_id == "prism1" and tf.header.frame_id == "base_link" and self.Q1 is None:
                self.T_prism1_to_base_link = self.tfTransform_to_matrix(tf.transform)
                self.Q1 = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                self.get_logger().info(f"Got prism1 transform: {self.Q1}")
            elif tf.child_frame_id == "prism2" and tf.header.frame_id == "base_link" and self.Q2 is None:
                self.Q2 = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                self.get_logger().info(f"Got prism2 transform: {self.Q2}")
            elif tf.child_frame_id == "prism3" and tf.header.frame_id == "base_link" and self.Q3 is None:
                self.Q3 = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                self.get_logger().info(f"Got prism3 transform: {self.Q3}")
            elif tf.child_frame_id == "base_link" and tf.header.frame_id == "odom" and self.T_odom_to_base_link is None:
                self.T_odom_to_base_link = self.tfTransform_to_matrix(tf.transform)
                self.get_logger().info(f"Got odom to base_link transform:\n {self.T_odom_to_base_link}")
            elif tf.child_frame_id == "odom" and tf.header.frame_id == "map" and self.T_map_to_odom is None:
                self.T_map_to_odom = self.tfTransform_to_matrix(tf.transform)
                self.get_logger().info(f"Got map to odom transform:\n {self.T_map_to_odom}")
        if self.Q1 is not None and self.Q2 is not None and self.Q3 is not None and self.T_odom_to_base_link is not None and self.T_map_to_odom is not None:
            self.tf_acquired = True
            self.destroy_subscription(self.tf_sub)
            self.destroy_subscription(self.tf_static_sub)

    def tfTransform_to_matrix(self, transform):
        T = np.eye(4)
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        q = transform.rotation
        T[0,0] = 1 - 2 * (q.y **2 + q.z **2)
        T[0,1] = 2 * (q.x * q.y - q.z * q.w)
        T[0,2] = 2 * (q.x * q.z - q.y * q.w)
        T[1,0] = 2 * (q.x * q.y + q.w * q.z)
        T[1,1] = 1 - 2 * (q.x **2 + q.z **2)
        T[1,2] = 2 * (q.x * q.z - q.w * q.x)
        T[2,0] = 2 * (q.x * q.z - q.w * q.y)
        T[2,1] = 2 * (q.y * q.z + q.w * q.x)
        T[2,2] = 1 - 2 * (q.x **2 + q.y **2)
        return T
    
    def update_calibration(self,position):
        if self.prism_id == -1 or self.euclidian_distance(position, self.prism_positions[self.prism_id]) > 0.3:
            self.prism_id += 1
            if self.prism_id < 3:
                self.prism_positions.append(position)
            self.current_sums = position
            self.nb_poses = 1
            if self.prism_id < 3:
                self.get_logger().info(f"New prism detected, now calibrating prism {self.prism_id+1}...")
        if self.prism_id < 3:
            self.current_sums += position
            self.nb_poses += 1
            self.prism_positions[self.prism_id] = self.current_sums / self.nb_poses
            self.get_logger().info(f"Prism {self.prism_id+1} position: {self.prism_positions[self.prism_id]}")

    def euclidian_distance(self, position1, position2):
        return np.sqrt(np.sum((position1 - position2) ** 2))
    
    def compute_calibration(self):
        while not self.tf_acquired:
            self.get_logger().info("Waiting for transforms...")
            time.sleep(0.5)
        P = np.array(self.prism_positions).T
        Q = np.array([self.Q1, self.Q2, self.Q3]).T
        P = np.vstack((P, np.ones((1, P.shape[1]))))
        Q = np.vstack((Q, np.ones((1, Q.shape[1]))))
        print("P\n:", P, "\nQ\n:", Q)
        self.T_base_link_to_theodo = self.minimization(P, Q)
        self.T_theodo_to_odom = self.T_odom_to_base_link @ self.T_base_link_to_theodo
        self.T_theodo_to_map = self.T_map_to_odom @ self.T_theodo_to_odom
        self.get_logger().info(f"Done! Publishing calibration matrix:\n{self.T_base_link_to_theodo}")
        self.timer.reset()

    def minimization(self, P, Q):
        mu_p = np.mean(P[0:3, :], axis=1)
        mu_q = np.mean(Q[0:3, :], axis=1)
        e_p = (P[:3].T - mu_p).T
        e_q = (Q[:3].T - mu_q).T
        H = e_p @ e_q.T
        U, s, VT = np.linalg.svd(H)
        M = np.eye(3)
        M[2, 2] = np.linalg.det(VT.T @ U.T)
        R = VT.T @ M @ U.T
        t = mu_q - R @ mu_p
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = t
        return T

    def publish_transform(self, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'theodolite'
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        q = np.zeros(4)
        q[0] = np.sqrt(1 + T[0, 0] + T[1, 1] + T[2, 2]) / 2
        q[1] = (T[2, 1] - T[1, 2]) / (4 * q[0])
        q[2] = (T[0, 2] - T[2, 0]) / (4 * q[0])
        q[3] = (T[1, 0] - T[0, 1]) / (4 * q[0])
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(t)
    
    def timer_callback(self):
        self.get_logger().info("Publishing calibration transform...")
        self.publish_transform(self.T_base_link_to_theodo)

    def publish_pose(self, position):
        position = np.array([position[0], position[1], position[2], 1])
        self.pose.pose.position.x = position[0]
        self.pose.pose.position.y = position[1]
        self.pose.pose.position.z = position[2]
        self.get_logger().info('Published Pose:')
        self.get_logger().info('Time: %d.%09d' % (self.pose.header.stamp.sec, self.pose.header.stamp.nanosec))
        self.get_logger().info('X: %f' % self.pose.pose.position.x)
        self.get_logger().info('Y: %f' % self.pose.pose.position.y)
        self.get_logger().info('Z: %f' % self.pose.pose.position.z)
        self.publisher.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    ground_truth_subscriber = GroundTruth()
    rclpy.spin(ground_truth_subscriber)
    ground_truth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
