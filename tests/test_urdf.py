
import math
import os

import pytest
import xacro
from urdf_parser_py.urdf import URDF

model_data = [
    ("med7", 25.5),
    ("med14", 32.3),
    ("iiwa7", 23.9),
    ("iiwa14", 29.9)
]

@pytest.mark.parametrize("model, mass", model_data)
def test_urdf_mass(model: str, mass: float, abs_tol: float=1.e-5) -> None:
    r"""Tests the total mass withtin the URDF files against
    the KUKA reference mass.

    Args:
        model (str): Model name
        mass (float): Reference mass
        abs_tol (float): Absolute tolerance in kg, 1.e-5 kg = 0.01 g
    """
    path = f"{os.getcwd()}/src/lbr_fri_ros2_stack/lbr_description/urdf/{model}/{model}.urdf.xacro"

    xml = xacro.process(path)
    urdf = URDF.from_xml_string(xml)

    mass_in_urdf = 0.

    for link in urdf.links:
        if link:
            if link.inertial:
                mass_in_urdf += link.inertial.mass

    if not math.isclose(mass, mass_in_urdf, abs_tol=abs_tol):
        raise ValueError(f"Expected robot mass of {mass} kg, found {mass_in_urdf} kg for model {model}.")
