#!/usr/bin/env python3
"""
Sine-Wave Joint State Publisher (20 Hz)
Two-link planar inspection arm and forward kinematics

Publishes continuous sinusoidal joint trajectories to /joint_states at 20 Hz.
Maintains joint positions strictly within revolute limits [-pi, +pi].
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SineWaveJointPublisher(Node):

    def __init__(self):
        super().__init__('sine_wave_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # 20 Hz publishing rate (period = 0.05 seconds)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.joint_names = ['joint1', 'joint2']
        self.start_time = self.get_clock().now()
        
        # Motion parameters: amplitudes well within [-pi, +pi] (-3.14159 to +3.14159)
        self.amplitude_q1 = 1.0   # rad (~57.3 deg)
        self.amplitude_q2 = 1.2   # rad (~68.8 deg)
        self.omega_q1 = 0.5       # rad/s (slow, smooth inspection sweep)
        self.omega_q2 = 0.8       # rad/s
        
        self.get_logger().info('SineWaveJointPublisher started. Publishing to /joint_states at 20 Hz.')

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        
        # Compute sinusoidal joint positions
        q1 = self.amplitude_q1 * math.sin(self.omega_q1 * t)
        q2 = self.amplitude_q2 * math.cos(self.omega_q2 * t)
        
        # Compute joint velocities (analytical derivatives)
        v1 = self.amplitude_q1 * self.omega_q1 * math.cos(self.omega_q1 * t)
        v2 = -self.amplitude_q2 * self.omega_q2 * math.sin(self.omega_q2 * t)
        
        # Build JointState message
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.joint_names
        msg.position = [q1, q2]
        msg.velocity = [v1, v2]
        msg.effort = [0.0, 0.0]
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
