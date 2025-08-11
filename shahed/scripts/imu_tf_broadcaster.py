#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        # Frame names (match URDF)
        self.parent_frame = "base_link"
        self.child_frame = "imu_frame"

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        # Position (zero for pure rotation)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Directly use IMU orientation
        t.transform.rotation = msg.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
