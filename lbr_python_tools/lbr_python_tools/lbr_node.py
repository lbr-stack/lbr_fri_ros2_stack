import os
import time
import optas
import rclpy
from rclpy import qos
from rclpy.node import Node
from copy import deepcopy
from ament_index_python import get_package_share_directory
from typing import List
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from lbr_fri_msgs.msg import LBRCommand, LBRState


class StateListener:
    def __init__(self, node):
        self._node = node
        self._t_prev = None
        self._t = None
        self._prev_prev_msg = None
        self._prev_msg = None
        self._msg = None
        if self._node.sim:
            self._topic_cls = JointState
            self._topic_name = "joint_states"
        else:
            self._topic_cls = LBRState
            self._topic_name = "lbr_states"

        self._sub = node.create_subscription(
            self._topic_cls,
            self._topic_name,
            self._callback,
            qos.qos_profile_system_default,
        )
        if self._node.sim:
            self.get_position = self.get_joint_states_position
            self.get_velocity = self.get_joint_states_velocity
            self.get_acceleration = self.get_joint_states_acceleration
        else:
            self.get_position = self.get_lbr_states_position
            self.get_velocity = self.get_lbr_states_velocity
            self.get_acceleration = self.get_lbr_states_acceleration

    def _callback(self, msg):
        self._prev_prev_msg = deepcopy(self._prev_msg)
        self._prev_msg = deepcopy(self._msg)
        self._msg = msg
        self._t_prev = deepcopy(self._t)
        self._t = time.time()

    def _get(self):
        return deepcopy(self._msg)

    def _get_prev(self):
        return deepcopy(self._prev_msg)

    def _get_prev_prev(self):
        return deepcopy(self._prev_prev_msg)

    def recieved(self):
        return (
            (self._msg is not None)
            and (self._prev_msg is not None)
            and (self._prev_prev_msg is not None)
        )

    def get_lbr_states_position(self):
        return self._get().measured_joint_position

    def get_lbr_states_velocity(self):
        curr = self._get().measured_joint_position
        prev = self._get_prev().measured_joint_position
        return [(c - p) / self._msg.sample_time for c, p in zip(curr, prev)]

    def get_lbr_states_acceleration(self):
        dt = self._msg.sample_time
        curr = optas.np.array(self._get().measured_joint_position)
        prev = optas.np.array(self._get_prev().measured_joint_position)
        prev_prev = optas.np.array(self._get_prev_prev().measured_joint_position)
        dcurr = (curr - prev) / dt
        dprev = (prev - prev_prev) / dt
        ddq = (dcurr - dprev) / dt
        return ddq.tolist()

    def _get_joint_states(self, msg, attr):
        # Ensure joint positions are in correct order
        q = []
        x = getattr(msg, attr)
        for joint_name in self._node.model.actuated_joint_names:
            joint_idx = msg.name.index(joint_name)
            q.append(x[joint_idx])
        return q

    def get_joint_states_position(self):
        return self._get_joint_states(self._get(), "position")

    def get_joint_states_velocity(self):
        return self._get_joint_states(self._get(), "velocity")

    def get_joint_states_acceleration(self):
        dt = self._t - self._t_prev
        dqcurr = optas.np.array(self._get_joint_states(self._get(), "velocity"))
        dqprev = optas.np.array(self._get_joint_states(self._get_prev(), "velocity"))
        ddq = (dqcurr - dqprev) / dt
        return ddq.tolist()


class RobotCommander:
    def __init__(self, node):
        self._node = node
        if self._node.sim:
            self._topic_cls = Float64MultiArray
            self._topic_name = "forward_position_controller/commands"
            self._pack = self._pack_float64multiarray
        else:
            self._topic_cls = LBRCommand
            self._topic_name = "lbr_command"
            self._pack = self._pack_lbr_command

        self._pub = node.create_publisher(
            self._topic_cls, self._topic_name, qos.qos_profile_system_default
        )

    def _pack_lbr_command(self, q: List[float]):
        return LBRCommand(joint_position=q)

    def _pack_float64multiarray(self, q: List[float]):
        return Float64MultiArray(data=q)

    def __call__(self, q: List[float]):
        self._pub.publish(self._pack(q))


class LBRPositionControlNode(Node):
    def __init__(self, node_name):
        # Initialize ros node
        super().__init__(node_name)

        # Get parameters
        self.declare_parameter("sim", True)
        self.declare_parameter("model", "med7")

        self.sim = bool(self.get_parameter("sim").value)
        self.model_name = str(self.get_parameter("model").value)

        # Initialize robot model
        self.xacro_filename = os.path.join(
            get_package_share_directory("lbr_description"),
            "urdf",
            self.model_name,
            f"{self.model_name}.urdf.xacro",
        )
        self.model = optas.RobotModel(xacro_filename=self.xacro_filename)

        # Start state listener and commander
        self.command = RobotCommander(self)
        self.joint_states = StateListener(self)
