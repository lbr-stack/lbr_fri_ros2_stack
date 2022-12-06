#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from lbr_fri_msgs.msg import LBRState

import numpy as np


class LBRStateSmoothingNode(Node):
    def __init__(self):
        super().__init__('lbr_state_smoothing_node')

        # Declare and get parameters
        self.declare_parameter('max_window_length', 20)
        self._max_window_length = int(self.get_parameter('max_window_length').value)
        self._window = []

        # Set filter method
        self._filter = self._filter_mean
        # self._filter = self._filter_median

        # Create publishers/subscribers
        self._lbr_state_smooth_publisher = self.create_publisher(LBRState, "/lbr_state/smooth", 1)
        self._lbr_state_callback = self.create_subscription(
            LBRState, '/lbr_state', self._lbr_state_callback, 1,
        )

    def _update_window(self, msg):
        self._window.append(msg)
        if len(self._window) > self._max_window_length:
            self.window.pop(0)

    def _filter_mean(self):
        smooth_state = self._window[-1]
        smooth_state.measured_joint_position = np.mean([m.measured_joint_position for m in self._window], axis=0).tolist()
        smooth_state.measured_torque = np.mean([m.measured_torque for m in self._window], axis=0).tolist()
        smooth_state.external_torque = np.mean([m.external_torque for m in self._window], axis=0).tolist()
        return smooth_state

    def _filter_median(self):
        smooth_state = self._window[-1]
        smooth_state.measured_joint_position = np.median([m.measured_joint_position for m in self._window], axis=0).tolist()
        smooth_state.measured_torque = np.median([m.measured_torque for m in self._window], axis=0).tolist()
        smooth_state.external_torque = np.median([m.external_torque for m in self._window], axis=0).tolist()
        return smooth_state

    def _lbr_state_callback(self, state):
        self._update_window(state)
        self._lbr_state_smooth_publisher.publish(self._filter())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LBRStateSmoothingNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
