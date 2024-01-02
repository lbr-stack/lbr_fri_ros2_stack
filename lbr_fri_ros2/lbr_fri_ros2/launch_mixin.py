from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class LBRFRIROS2Mixin:
    @staticmethod
    def arg_command_guard_variant() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="command_guard_variant",
            default_value="safe_stop",
            description="Command guard variant.",
            choices=["default", "safe_stop"],
        )

    @staticmethod
    def arg_open_loop() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="open_loop",
            default_value="true",
            description="Open loop control. Works best for LBRs. Should only be set to false by experienced users.",
        )

    @staticmethod
    def arg_rt_prio() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rt_prio",
            default_value="80",
            description="Realtime priority of the FRI thread. Realtime kernel required.\n"
            "\tRequires configuration in /etc/security/limits.conf. Add the line:\n"
            "\t'user - rtprio 99', where user is your username.",
        )

    @staticmethod
    def arg_pid_p() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="pid.p",
            default_value="1.0",
            description="Joint position PID controller proportional gain.",
        )

    @staticmethod
    def arg_pid_i() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="pid.i",
            default_value="0.0",
            description="Joint position PID controller integral gain.",
        )

    @staticmethod
    def arg_pid_d() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="pid.d",
            default_value="1.0",
            description="Joint position PID controller derivative gain.",
        )

    @staticmethod
    def arg_pid_i_max() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="pid.i_max",
            default_value="0.0",
            description="Joint position PID controller maximum integral value.",
        )

    @staticmethod
    def arg_pid_i_min() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="pid.i_min",
            default_value="0.0",
            description="Joint position PID controller minimum integral value.",
        )

    @staticmethod
    def arg_pid_antiwindup() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="pid.antiwindup",
            default_value="0.0",
            description="Joint position PID controller antiwindup.",
        )

    @staticmethod
    def arg_port_id() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="port_id",
            default_value="30200",
            description="Port ID of the FRI communication. Valid in range [30200, 30209].\n"
            "\tUsefull in multi-robot setups.",
        )

    @staticmethod
    def param_command_guard_variant() -> Dict[str, LaunchConfiguration]:
        return {
            "command_guard_variant": LaunchConfiguration(
                "command_guard_variant", default="safe_stop"
            )
        }

    @staticmethod
    def param_open_loop() -> Dict[str, LaunchConfiguration]:
        return {"open_loop": LaunchConfiguration("open_loop", default="true")}

    @staticmethod
    def param_rt_prio() -> Dict[str, LaunchConfiguration]:
        return {"rt_prio": LaunchConfiguration("rt_prio", default="80")}

    @staticmethod
    def param_pid_p() -> Dict[str, LaunchConfiguration]:
        return {"pid.p": LaunchConfiguration("pid.p", default="1.0")}

    @staticmethod
    def param_pid_i() -> Dict[str, LaunchConfiguration]:
        return {"pid.i": LaunchConfiguration("pid.i", default="0.0")}

    @staticmethod
    def param_pid_d() -> Dict[str, LaunchConfiguration]:
        return {"pid.d": LaunchConfiguration("pid.d", default="0.0")}

    @staticmethod
    def param_pid_i_max() -> Dict[str, LaunchConfiguration]:
        return {"pid.i_max": LaunchConfiguration("pid.i_max", default="0.0")}

    @staticmethod
    def param_pid_i_min() -> Dict[str, LaunchConfiguration]:
        return {"pid.i_min": LaunchConfiguration("pid.i_min", default="0.0")}

    @staticmethod
    def param_pid_antiwindup() -> Dict[str, LaunchConfiguration]:
        return {"pid.antiwindup": LaunchConfiguration("pid.antiwindup", default="0.0")}

    @staticmethod
    def param_port_id() -> Dict[str, LaunchConfiguration]:
        return {"port_id": LaunchConfiguration("port_id", default="30200")}

    @staticmethod
    def node_app(
        robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    ) -> DeclareLaunchArgument:
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name", default="lbr")
        return Node(
            package="lbr_fri_ros2",
            executable="app",
            namespace=robot_name,
            emulate_tty=True,
            output="screen",
            **kwargs,
        )
