from setuptools import find_packages, setup

package_name = "lbr_demos_moveit_python"

setup(
    name=package_name,
    version="1.4.3",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sequenced_motion = lbr_demos_moveit_python.sequenced_motion:main",
        ],
    },
)
