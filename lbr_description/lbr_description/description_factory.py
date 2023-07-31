import os
from typing import Dict

import xacro
from ament_index_python import get_package_share_directory


def description_factory(
    model: str = "iiwa7", robot_name: str = "lbr", sim: str = "true"
) -> Dict[str, str]:
    r"""Robot description factory for the LBR.

    Args:
        model (str): The LBR model in use. Defaults to "iiwa7". Choices are "iiwa7", "iiwa14", "med7", "med14".
        robot_name  (str): The name of the robot.
        sim (str): Whether to use the simulation or not.

    Returns:
        robot_description (Dict[str, str]): The robot description as a dictionary.
    """
    robot_description = {
        "robot_description": xacro.process(
            os.path.join(
                get_package_share_directory("lbr_description"),
                f"urdf/{model}/{model}.urdf.xacro",
            ),
            mappings={
                "robot_name": robot_name,
                "sim": sim,
            },
        )
    }

    return robot_description
