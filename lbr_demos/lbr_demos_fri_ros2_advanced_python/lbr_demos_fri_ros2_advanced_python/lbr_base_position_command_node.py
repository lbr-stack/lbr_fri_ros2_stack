import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRPositionCommand, LBRState


class LBRBasePositionCommandNode(Node):
    r"""Utility class for creating a base node for sending position commands to the KUKA LBRs.
    Retrieves update rate and robot description from the parameter servers.
    """

    _update_rate: int
    _dt: float
    _robot_description: str

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # retrieve parameters
        self._update_rate = self._get_update_rate()
        self._dt = 1.0 / float(self._update_rate)
        self._robot_description = self._get_robot_description()

        # publishers and subscribers
        self._lbr_state_sub = self.create_subscription(
            LBRState, "state", self._on_lbr_state, 1
        )
        self._lbr_position_command_pub = self.create_publisher(
            LBRPositionCommand,
            "command/joint_position",
            1,
        )

    def _get_update_rate(self) -> int:
        update_rate_client = self.create_client(
            GetParameters, "controller_manager/get_parameters"
        )
        while not update_rate_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error(
                    "Interrupted while waiting for the service. Exiting."
                )
                return None
            self.get_logger().info(
                f"Waiting for {update_rate_client.srv_name} service..."
            )
        future = update_rate_client.call_async(
            GetParameters.Request(names=["update_rate"])
        )
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Service call failed!")
            return None
        self._update_rate = future.result().values[0].integer_value
        self.get_logger().info(
            f"Received update rate: {self._update_rate} Hz from {update_rate_client.srv_name}"
        )
        return self._update_rate

    def _get_robot_description(self) -> str:
        robot_description_client = self.create_client(
            GetParameters, "robot_state_publisher/get_parameters"
        )
        while not robot_description_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error(
                    "Interrupted while waiting for the service. Exiting."
                )
                return None
            self.get_logger().info(
                f"Waiting for {robot_description_client.srv_name} service..."
            )
        future = robot_description_client.call_async(
            GetParameters.Request(names=["robot_description"])
        )
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Service call failed!")
            return None
        robot_description = future.result().values[0].string_value
        self.get_logger().info(
            f"Received robot description from {robot_description_client.srv_name}"
        )
        return robot_description

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        raise NotImplementedError
