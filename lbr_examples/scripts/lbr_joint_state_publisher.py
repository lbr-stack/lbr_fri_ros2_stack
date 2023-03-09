#!/usr/bin/python3
import rclpy
from rclpy import qos
from rclpy.node import Node

import numpy as np

from lbr_fri_msgs.msg import LBRState
from sensor_msgs.msg import JointState


class LBRJointStatePublisher(Node):
    def __init__(self):
        super().__init__("lbr_joint_state_publisher")
        self.msg = JointState(
            name=[f"lbr_joint_{i}" for i in range(7)],
            position=[0.0] * 7,
        )
        self.pub = self.create_publisher(
            JointState, "joint_states", qos.qos_profile_system_default
        )
        self.sub = self.create_subscription(
            LBRState, "lbr_state", self._callback, qos.qos_profile_system_default
        )

    def _callback(self, msg):
        dq = np.array(self.msg.position) - np.array(msg.measured_joint_position)
        dt = float(msg.sample_time)
        dqdt = dq / dt
        self.msg.velocity = dqdt.tolist()
        self.msg.position = msg.measured_joint_position
        self.msg.effort = msg.measured_torque
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LBRJointStatePublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
