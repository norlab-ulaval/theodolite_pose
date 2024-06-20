import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_msgs.msg import TFMessage
from tf2_ros.transform_listener import TransformListener
from theodolite_node_msgs.msg import TheodoliteCoordsStamped
from rclpy.qos import qos_profile_action_status_default
import time


class Calibration(Node):
    def __init__(self):
        super().__init__('calibration')
        self.prism_id = -1
        self.prism_positions = []
        self.current_sums = np.zeros(3)
        self.nb_poses = 0
        self.Q1, self.Q2, self.Q3 = None, None, None
        self.T_theodo_to_map = None
        self.T_base_link_to_map = None
        self.tf_acquired = False

        self.create_subscription(
            PoseStamped,
            '/theodolite_master/theodolite_pose',
            self.listener_callback,
            10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_sub = self.create_subscription(
            TFMessage, 
            "/tf_static", 
            self.static_tf_callback, 
            qos_profile_action_status_default      # Makes durability transient_local
        )

        self.timer = self.create_timer(2, self.timer_callback)
        self.timer.cancel()
        
    
    def listener_callback(self, msg):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if self.prism_id == -1 or self.euclidian_distance(position[0], position[1], position[2], self.prism_positions[self.prism_id][0], self.prism_positions[self.prism_id][1], self.prism_positions[self.prism_id][2]) > 0.3:
            self.prism_id += 1
            if self.prism_id == 3:
                self.get_logger().info("All prisms have been detected.")
                self.compute_calibration()
            self.prism_positions.append(position)
            self.current_sums = position
            self.nb_poses = 1
            self.get_logger().info(f"New prism detected, now calibrating prism {self.prism_id}...")
        else:
            self.current_sums += position
            self.nb_poses += 1
            self.prism_positions[self.prism_id] = self.current_sums / self.nb_poses

        self.get_logger().info(f"Prism {self.prism_id} position: {self.prism_positions[self.prism_id]}")
    
    def euclidian_distance(self, x1, y1, z1, x2, y2, z2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
    
    def static_tf_callback(self, tf_msg):
        for tf in tf_msg.transforms:
            if tf.child_frame_id == "prism1" and tf.header.frame_id == "base_link":
                self.Q1 = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                self.get_logger().info(f"Got prism1 transform: {self.Q1}")
            elif tf.child_frame_id == "prism2" and tf.header.frame_id == "base_link":
                self.Q2 = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                self.get_logger().info(f"Got prism2 transform: {self.Q2}")
            elif tf.child_frame_id == "prism3" and tf.header.frame_id == "base_link":
                self.Q3 = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                self.get_logger().info(f"Got prism3 transform: {self.Q3}")
            elif tf.child_frame_id == "base_link" and tf.header.frame_id == "map":
                self.T_base_link_to_map = np.array(tf.transform)
                self.get_logger().info(f"Got base_link to map transform: {self.T_base_link_to_map}")
        
        if self.Q1 is not None and self.Q2 is not None and self.Q3 is not None and self.T_base_link_to_map is not None:
            self.tf_acquired = True
            self.destroy_subscription(self.tf_sub)

    def compute_calibration(self):
        while not self.tf_acquired:
            self.get_logger().info("Waiting for prism transforms...")
            time.sleep(0.5)
        P = np.array(self.prism_positions)  # prisms in theodolite frame
        Q = np.array([self.Q1, self.Q2, self.Q3]).T     # prisms in robot frame
        P = np.vstack((P, np.ones((1, P.shape[1]))))
        Q = np.vstack((Q, np.ones((1, Q.shape[1]))))
        self.T_theodo_to_base_link = self.minimization(P.T, Q.T)
        self.T_theodo_to_map = self.T_base_link_to_map @ self.T_theodo_to_base_link
        self.get_logger().info(f"Done! Publishing calibration matrix: {self.T_theodo_to_map}")
        self.timer.reset()

    def publish_transform(self, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        # TODO: Make sure its not reversed
        t.header.frame_id = 'map'
        t.child_frame_id = 'theodolite'
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]

        # TODO: Validate quaternion computation
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

    def minimization(self, P, Q):
        # TODO: Make sure this works properly (visualize ?)
        errors_before = Q - P 
        mu_p = np.mean(P[0:3, :], axis=1)
        mu_q = np.mean(Q[0:3, :], axis=1)
        P_mu = np.ones((3, P.shape[1]))
        Q_mu = np.ones((3, Q.shape[1]))
        for i in range(0, P_mu.shape[1]):
            P_mu[0:3, i] = P[0:3, i] - mu_p
        for i in range(0, Q_mu.shape[1]):
            Q_mu[0:3, i] = Q[0:3, i] - mu_q
        H = P_mu @ Q_mu.T
        U, s, V = np.linalg.svd(H)
        M = np.eye(3)
        M[2, 2] = np.linalg.det(V.T @ U.T)
        R = V.T @ M @ U.T
        t = mu_q - R @ mu_p
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = t
        return T
    
    def timer_callback(self):
        self.get_logger().info("Publishing calibration transform...")
        self.publish_transform(self.T_theodo_to_map)

def main(args=None):
    rclpy.init(args=args)
    calibration = Calibration()
    rclpy.spin(calibration)
    calibration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
