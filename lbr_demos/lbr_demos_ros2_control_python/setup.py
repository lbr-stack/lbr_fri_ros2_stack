from setuptools import setup

package_name = "lbr_demos_ros2_control_python"

setup(
    name=package_name,
    version="1.4.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="Python demos for the LBR ros2_control integration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_trajectory_executioner_node = lbr_demos_ros2_control_python.joint_trajectory_executioner_node:main",
        ],
    },
)
