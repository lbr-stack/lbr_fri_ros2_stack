import math
import os
import xml.etree.ElementTree as ET
from typing import Tuple

import pytest
import xacro
from ament_index_python import get_package_share_directory
from urdf_parser_py.urdf import URDF

from .lbr_model_specifications import LBR_SPECIFICATIONS_DICT, LBRSpecification


@pytest.fixture
def setup_xml_and_reference(kuka_id: str) -> Tuple[str, LBRSpecification]:
    r"""Setup XML containing URDF and reference specifications.

    Args:
        kuka_id (str): The KUKA model ID.

    Return:
        (Tuple[str, LBRSpecification]): The URDF as XML string file to be tested against the LBRSpecification reference.
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
        f"{lbr_specification.name}.xacro",
    )

    xml = xacro.process(path)
    return xml, lbr_specification


@pytest.mark.parametrize("kuka_id", LBR_SPECIFICATIONS_DICT)
def test_mass(
    setup_xml_and_reference: Tuple[str, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    xml, lbr_specification = setup_xml_and_reference
    urdf = URDF.from_xml_string(xml)

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
    setup_xml_and_reference: Tuple[str, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    xml, lbr_specification = setup_xml_and_reference
    urdf = URDF.from_xml_string(xml)

    for joint in urdf.joints:
        if joint.type == "revolute":
            urdf_min_position = joint.limit.lower
            kuka_min_position = math.radians(
                lbr_specification.joint_limits[joint.name].min_position
            )
            if not math.isclose(urdf_min_position, kuka_min_position, abs_tol=abs_tol):
                raise ValueError(
                    f"Expected minimum joint position {kuka_min_position} rad, found {urdf_min_position} rad for model {lbr_specification.name} and joint {joint.name}."
                )

            urdf_max_position = joint.limit.upper
            kuka_max_position = math.radians(
                lbr_specification.joint_limits[joint.name].max_position
            )
            if not math.isclose(urdf_max_position, kuka_max_position, abs_tol=abs_tol):
                raise ValueError(
                    f"Expected maximum joint position {kuka_max_position} rad, found {urdf_max_position} rad for model {lbr_specification.name} and joint {joint.name}."
                )


@pytest.mark.parametrize("kuka_id", LBR_SPECIFICATIONS_DICT)
def test_velocity_limits(
    setup_xml_and_reference: Tuple[str, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    xml, lbr_specification = setup_xml_and_reference
    urdf = URDF.from_xml_string(xml)

    for joint in urdf.joints:
        if joint.type == "revolute":
            urdf_max_velocity = joint.limit.velocity
            kuka_max_velcoity = math.radians(
                lbr_specification.joint_limits[joint.name].max_velocity
            )
            if not math.isclose(urdf_max_velocity, kuka_max_velcoity, abs_tol=abs_tol):
                raise ValueError(
                    f"Expected minimum joint position {kuka_max_velcoity} rad/s, found {urdf_max_velocity} rad/s for model {lbr_specification.name} and joint {joint.name}."
                )


@pytest.mark.parametrize("kuka_id", LBR_SPECIFICATIONS_DICT)
def test_position_limits_ros2_control(
    setup_xml_and_reference: Tuple[str, LBRSpecification], abs_tol: float = 1.0e-5
) -> None:
    xml, lbr_specification = setup_xml_and_reference
    xml = ET.ElementTree(ET.fromstring(xml))
    for joint_interface in xml.find("ros2_control").iter("joint_interface"):
        for command_interface in joint_interface.iter("command_interface"):
            if command_interface.get("name") == "position":
                for param in command_interface.iter("param"):
                    if param.get("name") == "min":
                        urdf_min_position = float(param.text)
                        kuka_min_position = math.radians(
                            lbr_specification.joint_limits[
                                joint_interface.name
                            ].min_position
                        )
                        if not math.isclose(
                            urdf_min_position, kuka_min_position, abs_tol=abs_tol
                        ):
                            raise ValueError(
                                f"Expected minimum joint position {kuka_min_position} rad, found {urdf_min_position} rad for model {lbr_specification.name} and position command interface joint {joint_interface.name}."
                            )
                    elif param.get("name") == "max":
                        urdf_max_position = float(param.text)
                        kuka_max_position = math.radians(
                            lbr_specification.joint_limits[
                                joint_interface.name
                            ].max_position
                        )
                        if not math.isclose(
                            urdf_max_position, kuka_max_position, abs_tol=abs_tol
                        ):
                            raise ValueError(
                                f"Expected maximum joint position {kuka_max_position} rad, found {urdf_max_position} rad for model {lbr_specification.name} and position command interface joint {joint_interface.name}."
                            )
                    else:
                        raise ValueError("Couldn't find name.")
