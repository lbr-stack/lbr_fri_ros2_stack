from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class SequencedMotion(Node):
    def __init__(self):
        super().__init__("sequenced_motion")
        self._planning_scene_sub = self.create_subscription(
            PlanningScene, "/lbr/monitored_planning_scene", self._on_planning_scene, 1
        )
        self._robot_state = None

        self._action_client = ActionClient(
            self, MoveGroupSequence, "/lbr/sequence_move_group"
        )
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client._action_name}...")

        self._base = "link_0"
        self._end_effector = "link_ee"
        self._move_group_name = "arm"

    def _on_planning_scene(self, msg: PlanningScene) -> None:
        self._robot_state = msg.robot_state

    def _build_motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
        req = MotionPlanRequest()

        # general config
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"  # For Pilz PTP, LIN of CIRC
        req.allowed_planning_time = 10.0
        req.group_name = self._move_group_name
        req.max_acceleration_scaling_factor = 0.1
        req.max_velocity_scaling_factor = 0.01
        req.num_planning_attempts = 100

        # goal constraints
        req.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=self._base),
                        link_name=self._end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target_pose.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=self._base),
                        link_name=self._end_effector,
                        orientation=target_pose.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        return req

    def execute_sequence(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for idx, target_pose in enumerate(target_poses):
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=(
                        0.05 if idx != len(target_poses) - 1 else 0.0
                    ),  # last radius must be 0
                    req=self._build_motion_plan_request(target_pose),
                )
            )
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)


def main() -> None:
    rclpy.init()
    sequenced_motion = SequencedMotion()
    target_poses = [
        Pose(position=Point(x=0.0, y=0.0, z=1.0), orientation=Quaternion(w=1.0)),
        Pose(position=Point(x=0.2, y=0.2, z=0.8), orientation=Quaternion(w=1.0)),
    ]
    sequenced_motion.execute_sequence(target_poses=target_poses)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
