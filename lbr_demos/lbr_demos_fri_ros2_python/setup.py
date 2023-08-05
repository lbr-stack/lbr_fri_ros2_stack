from glob import glob

from setuptools import setup

package_name = "lbr_demos_fri_ros2_python"

setup(
    name=package_name,
    version="1.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="Standalone Python demos for the LBR.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_sine_overlay_node = lbr_demos_fri_ros2_python.joint_sine_overlay_node:main",
            "torque_sine_overlay_node = lbr_demos_fri_ros2_python.torque_sine_overlay_node:main",
            "wrench_sine_overlay_node = lbr_demos_fri_ros2_python.wrench_sine_overlay_node:main",
        ],
    },
)
