#!/usr/bin/python3
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRState


class LBRStateSmoothingNode(Node):
    def __init__(self) -> None:
        super().__init__("lbr_state_smoothing_node")

        # Declare and get parameters
        self.declare_parameter("max_window_length", 20)
        self.max_window_length_ = int(self.get_parameter("max_window_length").value)
        self.window_ = []

        # Set filter method
        self.filter_ = self.filter_mean_
        # self.filter_ = self.filter_median_

        # Create publishers/subscribers
        self.lbr_state_smooth_publisher_ = self.create_publisher(
            LBRState, "/lbr_state/smooth", 1
        )
        self.lbr_state_subscription_ = self.create_subscription(
            LBRState,
            "/lbr_state",
            self.lbr_state_callback_,
            1,
        )

    def update_window_(self, state: LBRState) -> None:
        self.window_.append(state)
        if len(self.window_) > self.max_window_length_:
            self.window_.pop(0)

    def get_most_recent_state_(self) -> LBRState:
        return deepcopy(self.window_[-1])

    def filter_mean_(self) -> LBRState:
        smooth_state = self.get_most_recent_state_()
        smooth_state.measured_joint_position = np.mean(
            [m.measured_joint_position for m in self.window_], axis=0
        ).tolist()
        smooth_state.measured_torque = np.mean(
            [m.measured_torque for m in self.window_], axis=0
        ).tolist()
        smooth_state.external_torque = np.mean(
            [m.external_torque for m in self.window_], axis=0
        ).tolist()
        return smooth_state

    def filter_median_(self) -> LBRState:
        smooth_state = self.get_most_recent_state_()
        smooth_state.measured_joint_position = np.median(
            [m.measured_joint_position for m in self.window_], axis=0
        ).tolist()
        smooth_state.measured_torque = np.median(
            [m.measured_torque for m in self.window_], axis=0
        ).tolist()
        smooth_state.external_torque = np.median(
            [m.external_torque for m in self.window_], axis=0
        ).tolist()
        return smooth_state

    def lbr_state_callback_(self, state: LBRState):
        self.update_window_(state)
        self.lbr_state_smooth_publisher_.publish(self.filter_())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LBRStateSmoothingNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
