import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_action_status_default

class ICP(Node):
    def __init__(self):
        super().__init__('icp_subscriber')
        self.input_topic = self.declare_parameter('input_topic', '/mapping/icp_odom').value
        self.output_topic = self.declare_parameter('output_topic', '/theodolite_master/icp_theodolite_pose').value
        self.create_subscription(
            Odometry,
            self.input_topic,
            self.listener_callback,
            10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, 
            "/tf_static", 
            self.tf_callback, 
            qos_profile_action_status_default
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10)
        
        self.prism1_position = None
        self.tf_acquired = False
    
    def tf_callback(self, tf_msg):
        for tf in tf_msg.transforms:
            if tf.child_frame_id == "prism1" and tf.header.frame_id == "base_link":
                self.prism1_position = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, 1])
                self.get_logger().info('Received T_prism1_to_base_link: %s' % self.prism1_position)
                self.tf_acquired = True
                self.destroy_subscription(self.tf_static_sub)

    def pose_to_matrix(self, pose):
        T = np.eye(4)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        q = pose.orientation
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
    
    def listener_callback(self, msg):
        if self.tf_acquired:
            self.pose = PoseStamped()
            self.T_base_link_to_map = self.pose_to_matrix(msg.pose.pose)
            position = self.T_base_link_to_map @ self.prism1_position
            self.pose.header = msg.header
            self.pose.pose.position.x = position[0]
            self.pose.pose.position.y = position[1]
            self.pose.pose.position.z = position[2]
            self.publisher.publish(self.pose)
            self.get_logger().info(f'Published {self.input_topic} to {self.output_topic} in base_link frame:')
            self.get_logger().info('Time: %d.%09d' % (self.pose.header.stamp.sec, self.pose.header.stamp.nanosec))
            self.get_logger().info('X: %f' % self.pose.pose.position.x)
            self.get_logger().info('Y: %f' % self.pose.pose.position.y)
            self.get_logger().info('Z: %f' % self.pose.pose.position.z)

def main(args=None):
    rclpy.init(args=args)
    icp = ICP()
    rclpy.spin(icp)
    icp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()