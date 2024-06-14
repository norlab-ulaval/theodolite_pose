import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from theodolite_node_msgs.msg import TheodoliteCoordsStamped
from theodolite_node_msgs.msg import TheodoliteTimeCorrection
from geometry_msgs.msg import Pose

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
            Pose,
            '/theodolite_master/theodolite_pose',
            10) 

    def listener_callback(self, msg):
        self.get_logger().info('Received Theodolite Data:')
        self.get_logger().info('Azimuth: %f' % msg.azimuth)
        self.get_logger().info('Elevation: %f' % msg.elevation)
        self.get_logger().info('Distance: %f' % msg.distance)
        self.publish_pose(msg.azimuth, msg.elevation, msg.distance)
    
    def publish_pose(self, azimuth, elevation, distance):
        pose = Pose()
        distance = distance + DISTANCE_DONE_BY_RASPBERRY_PI
        pose.position.x = distance * math.cos(-azimuth) * math.cos(np.pi/2-elevation)
        pose.position.y = distance * math.sin(-azimuth) * math.cos(np.pi/2-elevation)
        pose.position.z = distance * math.sin(np.pi/2-elevation)
        self.publisher.publish(pose)
        self.get_logger().info('Published Pose:')
        self.get_logger().info('X: %f' % pose.position.x)
        self.get_logger().info('Y: %f' % pose.position.y)
        self.get_logger().info('Z: %f' % pose.position.z)
    
    
def main(args=None):
    rclpy.init(args=args)
    ground_truth_subscriber = GroundTruth()
    rclpy.spin(ground_truth_subscriber)
    ground_truth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
