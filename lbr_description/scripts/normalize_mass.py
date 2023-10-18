import argparse

import numpy as np
import transformations as tf
import urchin


def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Normalize mass of URDF.")
    parser.add_argument(
        "--target_mass",
        type=float,
        help="Target mass of robot in kg.",
    )

    parser.add_argument(
        "--path",
        type=str,
        help="Path to URDF / xacro file.",
    )

    parser.add_argument(
        "--order",
        type=int,
        help="Order of magnitude for rounding of masses.",
    )

    return parser.parse_args()


def main() -> None:
    args = args_factory()

    target_mass = args.target_mass
    path = args.path
    order = args.order

    # load urdf
    urdf = urchin.URDF.load(path, lazy_load_meshes=True)

    print("Got actuated joints:")
    print(urdf.actuated_joint_names)

    def print_origin(origin: np.ndarray) -> None:
        xyz = tf.translation_from_matrix(origin)
        rpy = tf.euler_from_matrix(origin)
        xyz = np.around(xyz, decimals=order)
        rpy = np.around(rpy, decimals=order)
        print(f"xyz: {xyz} rpy: {rpy}")

    print("Inertial origins:")
    [print_origin(link.inertial.origin) for link in urdf.links]

    print("Got link masses:")
    link_masses = [link.inertial.mass for link in urdf.links]
    print(link_masses)

    total_mass = np.sum(link_masses)
    mass_ratio = target_mass / total_mass

    # correct masses
    corrected_link_masses = np.array(link_masses) * mass_ratio

    # 6 order magnitude
    corrected_link_masses = np.around(corrected_link_masses, decimals=order)

    print("Corrected link masses:")
    print(corrected_link_masses)

    # is close
    if not np.isclose(corrected_link_masses.sum(), target_mass, rtol=1e-5):
        raise ValueError(
            f"Masses do not sum up to target mass. Target mass: {target_mass} kg, sum of corrected masses: {corrected_link_masses.sum()}"
        )

    # correct inertias
    link_inertias = [link.inertial.inertia for link in urdf.links]
    corrected_link_inertias = [inertia * mass_ratio for inertia in link_inertias]

    # 6 order magnitude
    corrected_link_inertias = [
        np.around(inertia, decimals=order) for inertia in corrected_link_inertias
    ]

    def print_inertia(inertia: np.ndarray) -> None:
        print(
            f"ixx: {inertia[0, 0]} ixy: {inertia[0, 1]} ixz: {inertia[0, 2]} iyy: {inertia[1, 1]} iyz: {inertia[1, 2]} izz: {inertia[2, 2]}"
        )

    print("Corrected link inertias:")
    [print_inertia(inertia) for inertia in corrected_link_inertias]


if __name__ == "__main__":
    main()
