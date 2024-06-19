import numpy as np
import pandas as pd
from tf2utilities.main import TF2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from theodolite_node_msgs.msg import TheodoliteCoordsStamped

class Calibration(Node):
    def __init__(self):
        super().__init__('calibration')
        self.prism_id = -1
        self.prism_positions = []
        self.current_sums = np.zeros(3)
        self.nb_poses = 0
        self.create_subscription(
            PoseStamped,
            '/theodolite_master/theodolite_pose',
            self.listener_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def listener_callback(self, msg):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if self.prism_id == -1 or self.euclidian_distance(position[0], position[1], position[2], self.prism_positions[self.prism_id][0], self.prism_positions[self.prism_id][1], self.prism_positions[self.prism_id][2]) > 0.3:
            self.prism_id += 1
            if self.prism_id == 3:
                self.compute_calibration()
            self.prism_positions.append(position)
            self.current_sums = position
            self.nb_poses = 1
        else:
            self.current_sums += position
            self.nb_poses += 1
            self.prism_positions[self.prism_id] = self.current_sums / self.nb_poses
    
    def euclidian_distance(self, x1, y1, z1, x2, y2, z2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
    
    def get_prisms_in_base_link(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        try:
            t1 = self.tf_buffer.lookup_transform("prism1", "base_link", rclpy.time.Time())
            Q1 = np.array([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z])
            t2 = self.tf_buffer.lookup_transform("prism2", "base_link", rclpy.time.Time())
            Q2 = np.array([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z])
            t3 = self.tf_buffer.lookup_transform("prism3", "base_link", rclpy.time.Time())
            Q3 = np.array([t3.transform.translation.x, t3.transform.translation.y, t3.transform.translation.z])
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {"prism1"} to {"base_link"}: {ex}')
        return np.array(Q1, Q2, Q3)

    def compute_calibration(self):
        Q = self.get_prisms_in_base_link() #prisms in base_link frame
        P = [self.prism_positions] #prisms in theodolite frame
        P = np.vstack((P, np.ones((1, P.shape[1]))))
        Q = np.vstack((Q, np.ones((1, Q.shape[1]))))
        T = self.minimization(P.T, Q.T)
        self.publish_transform(T)
        self.get_logger().info('Shutting Down...')
        self.shutdown()

    def publish_transform(self, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        q = TF2.rotation_matrix_to_quaternion(T[0:3, 0:3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        t.child_frame_id = 'theodolite'
        self.tf_broadcaster.sendTransform(t)

    def minimization(P, Q):
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
