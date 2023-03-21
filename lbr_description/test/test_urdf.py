import math
import os
from typing import Tuple

import pytest
import xacro
from ament_index_python import get_package_share_directory
from urdf_parser_py.urdf import URDF, Robot

from .lbr_model_specifications import (
    LBR_SPECIFICATIONS_DICT,
    URDF_TO_KUKA_JOINT_NAME_DICT,
    LBRSpecification,
)


@pytest.fixture
def setup_urdf_and_reference(kuka_id: str) -> Tuple[Robot, LBRSpecification]:
    r"""Setup URDF and reference specifications.

    Args:
        kuka_id (str): The KUKA model ID.

    Return:
        (Tuple[Robot, LBRSpecification]): The URDF file to be tested against the LBRSpecification reference.
    """
    lbr_specification = LBR_SPECIFICATIONS_DICT[kuka_id]

    if lbr_specification.kuka_id != kuka_id:
        raise ValueError(
            f"Expected KUKA ID {kuka_id}, found {lbr_specification.kuka_id}."
        )

    path = os.path.join(
        get_package_share_directory("lbr_description"),
        "urdf",
        lbr_specification.name,
        f"{lbr_specification.name}.urdf.xacro",
    )

    xml = xacro.process(path)
    urdf = URDF.from_xml_string(xml)
    return urdf, lbr_specification


@pytest.mark.parametrize("kuka_id", LBR_SPECIFICATIONS_DICT)
def test_mass(
    setup_urdf_and_reference: Tuple[Robot, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    urdf, lbr_specification = setup_urdf_and_reference
    mass_in_urdf = 0.0
    for link in urdf.links:
        if link:
            if link.inertial:
                mass_in_urdf += link.inertial.mass

    if not math.isclose(lbr_specification.mass, mass_in_urdf, abs_tol=abs_tol):
        raise ValueError(
            f"Expected robot mass of {lbr_specification.mass} kg, found {mass_in_urdf} kg for model {lbr_specification.name}."
        )


@pytest.mark.parametrize("kuka_id", LBR_SPECIFICATIONS_DICT)
def test_position_limits(
    setup_urdf_and_reference: Tuple[Robot, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    urdf, lbr_specification = setup_urdf_and_reference

    for joint in urdf.joints:
        if joint.type == "revolute":
            urdf_joint_name = "_".join(joint.name.split("_")[-2:])
            kuka_joint_name = URDF_TO_KUKA_JOINT_NAME_DICT[urdf_joint_name]

            urdf_min_position = joint.limit.lower
            kuka_min_position = math.radians(
                lbr_specification.joint_limits[kuka_joint_name].min_position
            )
            if not math.isclose(urdf_min_position, kuka_min_position, abs_tol=abs_tol):
                raise ValueError(
                    f"Expected minimum joint position {kuka_min_position} rad, found {urdf_min_position} rad for model {lbr_specification.name}."
                )

            urdf_max_position = joint.limit.upper
            kuka_max_position = math.radians(
                lbr_specification.joint_limits[kuka_joint_name].max_position
            )
            if not math.isclose(urdf_max_position, kuka_max_position, abs_tol=abs_tol):
                raise ValueError(
                    f"Expected maximum joint position {kuka_max_position} rad, found {urdf_max_position} rad for model {lbr_specification.name}."
                )


@pytest.mark.parametrize("kuka_id", LBR_SPECIFICATIONS_DICT)
def test_velocity_limits(
    setup_urdf_and_reference: Tuple[Robot, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    urdf, lbr_specification = setup_urdf_and_reference

    for joint in urdf.joints:
        if joint.type == "revolute":
            urdf_joint_name = "_".join(joint.name.split("_")[-2:])
            kuka_joint_name = URDF_TO_KUKA_JOINT_NAME_DICT[urdf_joint_name]

            urdf_max_velocity = joint.limit.velocity
            kuka_max_velcoity = math.radians(
                lbr_specification.joint_limits[kuka_joint_name].max_velocity
            )
            if not math.isclose(urdf_max_velocity, kuka_max_velcoity, abs_tol=abs_tol):
                raise ValueError(
                    f"Expected minimum joint position {kuka_max_velcoity} rad/s, found {urdf_max_velocity} rad/s for model {lbr_specification.name}."
                )
