#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from vn_driver.msg import Vectornav

class DualIMUSubscriber(Node):
    def __init__(self):
        super().__init__('dual_imu_subscriber')
        
        # Configure QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers for both IMU topics
        self.imu1_subscription = self.create_subscription(
            Vectornav,
            'imu1',
            self.imu1_callback,
            qos_profile
        )
        
        self.imu2_subscription = self.create_subscription(
            Vectornav,
            'imu2',
            self.imu2_callback,
            qos_profile
        )
        
        # Initialize data storage if needed
        self.imu1_data = None
        self.imu2_data = None
        
        # Counter for received messages
        self.imu1_msg_count = 0
        self.imu2_msg_count = 0
        
        self.get_logger().info('Dual IMU Subscriber initialized')
        self.get_logger().info('Listening to topics: /imu1 and /imu2')
    
    def imu1_callback(self, msg):
        """Callback function for IMU1 topic"""
        self.imu1_data = msg
        self.imu1_msg_count += 1
        
        # Process IMU1 data here
        self.process_imu1_data(msg)
        
        # Optional: Check for synchronized data
        if self.imu1_data is not None and self.imu2_data is not None:
            self.process_synchronized_data()
    
    def imu2_callback(self, msg):
        """Callback function for IMU2 topic"""
        self.imu2_data = msg
        self.imu2_msg_count += 1
        
        # Process IMU2 data here
        self.process_imu2_data(msg)
        
        # Optional: Check for synchronized data
        if self.imu1_data is not None and self.imu2_data is not None:
            self.process_synchronized_data()
    
    def process_imu1_data(self, msg):
        """Process data from IMU1"""
        # Example: Extract and log orientation data
        # Adjust these fields based on your actual Vectornav message structure
        
        # If your message has a header
        if hasattr(msg, 'header'):
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().debug(f'IMU1 timestamp: {timestamp:.3f}')
        
        # Example fields - adjust based on your Vectornav.msg structure
        if hasattr(msg, 'orientation'):
            self.get_logger().info(
                f'IMU1 - Orientation: x={msg.orientation.x:.4f}, '
                f'y={msg.orientation.y:.4f}, z={msg.orientation.z:.4f}, '
                f'w={msg.orientation.w:.4f}'
            )
        
        # Log angular velocity if available
        if hasattr(msg, 'angular_velocity'):
            self.get_logger().debug(
                f'IMU1 - Angular velocity: x={msg.angular_velocity.x:.4f}, '
                f'y={msg.angular_velocity.y:.4f}, z={msg.angular_velocity.z:.4f}'
            )
        
        # Log linear acceleration if available
        if hasattr(msg, 'linear_acceleration'):
            self.get_logger().debug(
                f'IMU1 - Linear acceleration: x={msg.linear_acceleration.x:.4f}, '
                f'y={msg.linear_acceleration.y:.4f}, z={msg.linear_acceleration.z:.4f}'
            )
    
    def process_imu2_data(self, msg):
        """Process data from IMU2"""
        # Similar processing for IMU2
        
        if hasattr(msg, 'header'):
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().debug(f'IMU2 timestamp: {timestamp:.3f}')
        
        if hasattr(msg, 'orientation'):
            self.get_logger().info(
                f'IMU2 - Orientation: x={msg.orientation.x:.4f}, '
                f'y={msg.orientation.y:.4f}, z={msg.orientation.z:.4f}, '
                f'w={msg.orientation.w:.4f}'
            )
        
        if hasattr(msg, 'angular_velocity'):
            self.get_logger().debug(
                f'IMU2 - Angular velocity: x={msg.angular_velocity.x:.4f}, '
                f'y={msg.angular_velocity.y:.4f}, z={msg.angular_velocity.z:.4f}'
            )
        
        if hasattr(msg, 'linear_acceleration'):
            self.get_logger().debug(
                f'IMU2 - Linear acceleration: x={msg.linear_acceleration.x:.4f}, '
                f'y={msg.linear_acceleration.y:.4f}, z={msg.linear_acceleration.z:.4f}'
            )
    
    def process_synchronized_data(self):
        """Process synchronized data from both IMUs"""
        # This method is called when both IMU data are available
        # You can implement sensor fusion or comparison logic here
        
        # Example: Compare orientations
        if hasattr(self.imu1_data, 'orientation') and hasattr(self.imu2_data, 'orientation'):
            # Calculate difference in quaternion values (simplified)
            diff_x = abs(self.imu1_data.orientation.x - self.imu2_data.orientation.x)
            diff_y = abs(self.imu1_data.orientation.y - self.imu2_data.orientation.y)
            diff_z = abs(self.imu1_data.orientation.z - self.imu2_data.orientation.z)
            diff_w = abs(self.imu1_data.orientation.w - self.imu2_data.orientation.w)
            
            self.get_logger().debug(
                f'Orientation difference - x:{diff_x:.4f}, y:{diff_y:.4f}, '
                f'z:{diff_z:.4f}, w:{diff_w:.4f}'
            )
    
    def get_status(self):
        """Return the status of both subscribers"""
        return {
            'imu1_msg_count': self.imu1_msg_count,
            'imu2_msg_count': self.imu2_msg_count,
            'imu1_has_data': self.imu1_data is not None,
            'imu2_has_data': self.imu2_data is not None
        }


def main(args=None):
    rclpy.init(args=args)
    
    dual_imu_subscriber = DualIMUSubscriber()
    
    try:
        rclpy.spin(dual_imu_subscriber)
    except KeyboardInterrupt:
        # Print final statistics
        status = dual_imu_subscriber.get_status()
        dual_imu_subscriber.get_logger().info(
            f"\nShutting down...\n"
            f"Final statistics:\n"
            f"  IMU1 messages received: {status['imu1_msg_count']}\n"
            f"  IMU2 messages received: {status['imu2_msg_count']}"
        )
    finally:
        dual_imu_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()