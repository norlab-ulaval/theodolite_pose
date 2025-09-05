import time
import numpy as np
import rclpy
from rclpy.node import Node
from theodolite_node_msgs.msg import TheodoliteCoordsStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

PRISM_CONSTANT = 0.01

class GroundTruth(Node):

    def __init__(self):

        super().__init__('ground_truth_subscriber')
        self.input_topic = self.declare_parameter('input_topic', '/theodolite_data').value
        self.output_topic = self.declare_parameter('output_topic', '/theodolite_pose').value
        self.reference_frame = self.declare_parameter('reference_frame', 'trajectory').value
        self.robot_frame = self.declare_parameter('robot_frame', 'prism3').value
        self.min_measurements = self.declare_parameter('min_measurements', 3).value

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Perform initial calibration
        self.initial_TF = None
        self.perform_calibration()

        # Subscriptions and publishers
        self.create_subscription(TheodoliteCoordsStamped, self.input_topic, self.measurements_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 10)
    

    def perform_calibration(self):
        
        prisms_tfs = self.get_prisms_transforms()
        prism_measurements = self.get_prism_measurements()
        self.compute_initial_transform(prisms_tfs, prism_measurements)


    def get_prisms_transforms(self):

        prisms_tfs = []

        while True:
            try:
                prism1_transform = self.tf_buffer.lookup_transform(self.robot_frame, 'prism1', rclpy.time.Time())
                prism2_transform = self.tf_buffer.lookup_transform(self.robot_frame, 'prism2', rclpy.time.Time())
                prism3_transform = self.tf_buffer.lookup_transform(self.robot_frame, 'prism3', rclpy.time.Time())
                prisms_tfs = [prism1_transform, prism2_transform, prism3_transform]
                break
            except Exception as e:
                self.get_logger().warn("Waiting for prism transforms...")
                time.sleep(1.0)

        self.get_logger().info("Required transforms acquired.")
        return prisms_tfs


    def get_prism_measurements(self):

        prism_measurements = []

        # TODO: When migrating a newer ROS2 version, replace with rclpy.wait_for_message
        temp_sub = self.create_subscription(TheodoliteCoordsStamped, self.input_topic, self.temp_callback, 10)
        self.measurement_acquired = False

        for i in range(3):
            self.get_logger().info(f"Please point total station to prism {i+1}...")
            measurements = []
            while len(measurements) < self.min_measurements:
                # Wait for measurement with a small spin
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.measurement_acquired and self.temp_msg.status == 0:
                    position = self.rts_to_cartesian(self.temp_msg.distance, self.temp_msg.azimuth, self.temp_msg.elevation)

                    # Case for the first measurement
                    if (i == 0 and len(measurements) == 0):
                        measurements.append(position)
                        self.get_logger().info(f"Measurement {len(measurements)}/{self.min_measurements} for prism {i+1}: {position}")
                        self.measurement_acquired = False
                    
                    # Case for subsequent measurements (make sure we are on the same prism)
                    elif len(measurements) > 0 and self.euclidian_distance(position, measurements[-1]) < 0.03:
                        measurements.append(position)
                        self.get_logger().info(f"Measurement {len(measurements)}/{self.min_measurements} for prism {i+1}: {position}")
                        self.measurement_acquired = False

                    # Case for the first measurement of subsequent prisms (make sure we changed prism)
                    elif i > 0 and len(measurements) == 0 and self.euclidian_distance(position, prism_measurements[-1]) > 0.3:
                        measurements.append(position)
                        self.get_logger().info(f"Measurement {len(measurements)}/{self.min_measurements} for prism {i+1}: {position}")
                        self.measurement_acquired = False

            avg_position = np.mean(measurements, axis=0)
            prism_measurements.append(avg_position)
            self.get_logger().info(f"Prism {i+1} averaged position: {avg_position}")

        self.destroy_subscription(temp_sub)
        return np.array(prism_measurements)
    

    def temp_callback(self, msg):
        self.temp_msg = msg
        self.measurement_acquired = True
    

    def euclidian_distance(self, position1, position2):
        return np.sqrt(np.sum((position1 - position2) ** 2))
    

    def compute_initial_transform(self, prisms_tfs, prism_measurements):
        
        P = prism_measurements.T  # Theodolite measurements
        Q = np.array([
            [prisms_tfs[0].transform.translation.x, prisms_tfs[0].transform.translation.y, prisms_tfs[0].transform.translation.z],
            [prisms_tfs[1].transform.translation.x, prisms_tfs[1].transform.translation.y, prisms_tfs[1].transform.translation.z],
            [prisms_tfs[2].transform.translation.x, prisms_tfs[2].transform.translation.y, prisms_tfs[2].transform.translation.z]
        ]).T  # TF positions in base_link
        
        # Add homogeneous coordinates
        P = np.vstack((P, np.ones((1, P.shape[1]))))
        Q = np.vstack((Q, np.ones((1, Q.shape[1]))))
        
        # Compute transform from theodolite to base_link
        self.initial_TF = self.minimization(P, Q)
        self.get_logger().info("Initial calibration transform computed successfully!")
            

    def measurements_callback(self, msg):

        if msg.status == 0:
            position = self.rts_to_cartesian(msg.distance, msg.azimuth, msg.elevation)
            
            # Apply initial transform to align with reference frame
            position_homogeneous = np.array([position[0], position[1], position[2], 1])
            transformed_position = self.initial_TF @ position_homogeneous
            self.publish_pose(transformed_position[:3], msg.header.stamp)


    def rts_to_cartesian(self, distance, azimuth, elevation):

        distance = distance + PRISM_CONSTANT
        x = distance * np.cos(np.pi/2 - azimuth) * np.sin(elevation)
        y = distance * np.sin(np.pi/2 - azimuth) * np.sin(elevation)
        z = distance * np.cos(elevation)
        return np.array([x, y, z])


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
    

    def publish_pose(self, position, timestamp):

        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        
        timestamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        self.get_logger().info(f"Publishing Pose: Time: {timestamp:.4f}, X: {pose.pose.position.x:.4f}, Y: {pose.pose.position.y:.4f}, Z: {pose.pose.position.z:.4f}")
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    ground_truth_subscriber = GroundTruth()
    rclpy.spin(ground_truth_subscriber)
    ground_truth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
