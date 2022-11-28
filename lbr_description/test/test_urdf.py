import math
import os

import pytest
import xacro
from ament_index_python import get_package_share_directory
from urdf_parser_py.urdf import URDF

# masses from https://xpert.kuka.com (login required) or
# med masses: https://www.kuka.com/-/media/kuka-downloads/imported/9cb8e311bfd744b4b0eab25ca883f6d3/kuka_lbr-med_en.pdf?rev=13187739c0874dab8733f9ed7e73bc8a&hash=26DA2F0C35C02499CD3C066AB36DC7F6
# iiwa masses: https://www.kuka.com/-/media/kuka-downloads/imported/9cb8e311bfd744b4b0eab25ca883f6d3/kuka_lbr_iiwa_brochure_en.pdf?rev=b4399dedb61748a1910fc046c0974aa3&hash=8CFF4E315CB0CAB16CEBA0D6FE0155C2
model_data = [
    ("med7", 25.5),
    ("med14", 32.3),
    ("iiwa7", 23.9),
    ("iiwa14", 29.9)
]

@pytest.mark.parametrize("model, mass", model_data)
def test_urdf_mass(model: str, mass: float, abs_tol: float=1.e-5) -> None:
    r"""Compares the total mass withtin the URDF files against
    the KUKA reference mass.

    Args:
        model (str): Model name
        mass (float): Reference mass
        abs_tol (float): Absolute tolerance in kg, 1.e-5 kg = 0.01 g
    """
    path = os.path.join(get_package_share_directory("lbr_description"), "urdf", model, f"{model}.urdf.xacro")

    xml = xacro.process(path)
    urdf = URDF.from_xml_string(xml)

    mass_in_urdf = 0.

    for link in urdf.links:
        if link:
            if link.inertial:
                mass_in_urdf += link.inertial.mass

    if not math.isclose(mass, mass_in_urdf, abs_tol=abs_tol):
        raise ValueError(f"Expected robot mass of {mass} kg, found {mass_in_urdf} kg for model {model}.")
