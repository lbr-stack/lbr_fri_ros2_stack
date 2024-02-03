import glob

from setuptools import setup

package_name = "lbr_demos_fri_ros2_advanced_python"

setup(
    name=package_name,
    version="1.4.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="Advanced Python demos for the lbr_fri_ros2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "admittance_control_node = lbr_demos_fri_ros2_advanced_python.admittance_control_node:main",
        ],
    },
)
