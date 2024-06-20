import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from theodolite_node_msgs.msg import TheodoliteCoordsStamped
from theodolite_node_msgs.msg import TheodoliteTimeCorrection
from geometry_msgs.msg import PoseStamped

DISTANCE_DONE_BY_RASPBERRY_PI = 0.01

class GroundTruth(Node):

    def __init__(self):
        super().__init__('ground_truth_subscriber')
        self.create_subscription(
            TheodoliteCoordsStamped,
            '/theodolite_master/theodolite_data',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            PoseStamped,
            '/theodolite_master/theodolite_pose',
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received Theodolite Data:')
        self.get_logger().info('Status: %d' % msg.status)
        self.get_logger().info('Time: %d.%09d' % (msg.header.stamp.sec, msg.header.stamp.nanosec))
        self.get_logger().info('Azimuth: %f' % msg.azimuth)
        self.get_logger().info('Elevation: %f' % msg.elevation)
        self.get_logger().info('Distance: %f' % msg.distance)
        if msg.status == 0:
            self.publish_pose(msg.header.stamp.sec, msg.header.stamp.nanosec, msg.azimuth, msg.elevation, msg.distance)
    
    def publish_pose(self, sec, nanosec, azimuth, elevation, distance):
        pose = PoseStamped()
        distance = distance + DISTANCE_DONE_BY_RASPBERRY_PI
        pose.header.frame_id = 'theodolite'
        pose.header.stamp.sec = sec
        pose.header.stamp.nanosec = nanosec
        pose.pose.position.x = distance * np.cos(np.pi/2 - azimuth) * np.sin(elevation)
        pose.pose.position.y = distance * np.sin(np.pi/2 - azimuth) * np.sin(elevation)
        pose.pose.position.z = distance * np.cos(elevation)
        self.publisher.publish(pose)
        self.get_logger().info('Published Pose:')
        self.get_logger().info('Time: %d.%09d' % (pose.header.stamp.sec, pose.header.stamp.nanosec))
        self.get_logger().info('X: %f' % pose.pose.position.x)
        self.get_logger().info('Y: %f' % pose.pose.position.y)
        self.get_logger().info('Z: %f' % pose.pose.position.z)
    
def main(args=None):
    rclpy.init(args=args)
    ground_truth_subscriber = GroundTruth()
    rclpy.spin(ground_truth_subscriber)
    ground_truth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
