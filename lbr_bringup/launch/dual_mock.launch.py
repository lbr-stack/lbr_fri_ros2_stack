from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="ctrl",
            default_value="joint_trajectory_controller",
            description="Desired default controller.",
            choices=["joint_trajectory_controller"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr_dual_arm",
            description="Namespace for the dual-arm bringup nodes.",
        )
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_dual_arm_description"),
                        "urdf",
                        "lbr_dual_arm.xacro",
                    ]
                ),
                " mode:=mock",
            ]
        )
    }

    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description,
        robot_name=LaunchConfiguration("robot_name"),
        use_sim_time=False,
    )
    ld.add_action(robot_state_publisher)

    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        robot_name=LaunchConfiguration("robot_name"),
        use_sim_time=False,
        robot_description=robot_description,
        ctrl_cfg_pkg="lbr_dual_arm_description",
        ctrl_cfg="ros2_control/dual_arm_controllers.yaml",
    )
    ld.add_action(ros2_control_node)

    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="joint_state_broadcaster",
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller=LaunchConfiguration("ctrl"),
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster, controller],
        )
    )
    ld.add_action(controller_event_handler)
    return ld
