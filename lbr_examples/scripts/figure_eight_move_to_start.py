#!/usr/bin/python3
import sys
import time
import optas
import rclpy
from lbr_python_tools.lbr_node import LBRPositionControlNode


class FigureEightMoveToStartNode(LBRPositionControlNode):
    def __init__(self):
        super().__init__("figure_eight_move_to_start")
        self._start_joint_positon = (
            optas.deg2rad([0, 45, 0, -90, 0, -45, 0]).toarray().flatten()
        )
        self.declare_parameter("duration", 5.0)  # seconds
        self._duration = float(self.get_parameter("duration").value)
        self.declare_parameter("hz", 100)
        hz = int(self.get_parameter("hz").value)
        dt = 1.0 / float(hz)
        self._first = True
        self._timer = self.create_timer(dt, self._timer_callback)

    def _timer_callback(self):
        if not self.joint_states.recieved():
            return

        if self._first:
            self._current_joint_state = optas.np.array(self.joint_states.get_position())
            self._start_time = time.time()
            self._first = False

        t = time.time() - self._start_time
        alpha = t / self._duration
        q = (
            1.0 - alpha
        ) * self._current_joint_state + alpha * self._start_joint_positon

        if alpha > 1.0:
            self.command(self._start_joint_positon)  # avoid overshooting
            self.destroy_timer(self._timer)
            self.get_logger().info("robot at figure of eight starting position")
            sys.exit(0)
        else:
            self.command(q.tolist())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FigureEightMoveToStartNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
