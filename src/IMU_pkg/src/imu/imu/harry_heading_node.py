#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

def quat_to_euler(qx, qy, qz, qw):
    # roll
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (qw*qy - qz*qx)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class SpatialPublisher(Node):
    def __init__(self):
        super().__init__('spatial_publisher')
        self.declare_parameter('imu_topic', '/imu/data')  # <-- listen to quaternion topic
        self.declare_parameter('yaw_offset_deg', 80.0)    # 235-37-53-65

        self.pub = self.create_publisher(Float64, 'phidget', 10)
        topic = self.get_parameter('imu_topic').value
        self.sub = self.create_subscription(Imu, topic, self.imu_cb, 10)
        self.yaw_offset = float(self.get_parameter('yaw_offset_deg').value)
        self.get_logger().info(f'Heading node listening on {topic}, publishing /phidget')

    def imu_cb(self, msg: Imu):
        # Skip if orientation unset (common in /imu/data_raw)
        if msg.orientation_covariance[0] < 0.0:
            return
        q = msg.orientation
        _, _, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        heading = math.degrees(yaw) + self.yaw_offset
        self.pub.publish(Float64(data=heading))

def main():
    rclpy.init()
    node = SpatialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
