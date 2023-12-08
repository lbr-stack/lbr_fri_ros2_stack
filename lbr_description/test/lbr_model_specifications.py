from dataclasses import dataclass
from typing import Dict, List, Type


@dataclass
class JointLimit:
    min_position: float
    max_position: float
    max_velocity: float

    def __init__(
        self,
        min_position: float,
        max_position: float,
        max_velocity: float,
    ) -> None:
        self.min_position = min_position
        self.max_position = max_position
        self.max_velocity = max_velocity


@dataclass
class LBRSpecification:
    dof: int
    name: str
    kuka_id: str
    mass: float
    joint_limits: List[JointLimit]

    def __init__(
        self, name: str, kuka_id: str, mass: float, joint_limits: Dict[str, JointLimit]
    ) -> None:
        self.dof = 7
        self.name = name
        self.kuka_id = kuka_id
        self.mass = mass
        if len(joint_limits) != self.dof:
            raise ValueError(f"Expected {self.dof} dof, got {len(joint_limits)}.")
        self.joint_limits = joint_limits


# specifications as extracted from https://xpert.kuka.com/
LBR_SPECIFICATIONS_DICT: Dict[str, Type[LBRSpecification]] = {
    "AR7606": LBRSpecification(
        name="iiwa7",
        kuka_id="AR7606",
        mass=23.9,
        joint_limits={
            "A1": JointLimit(min_position=-170, max_position=170, max_velocity=98),
            "A2": JointLimit(min_position=-120, max_position=120, max_velocity=98),
            "A3": JointLimit(min_position=-170, max_position=170, max_velocity=100),
            "A4": JointLimit(min_position=-120, max_position=120, max_velocity=130),
            "A5": JointLimit(min_position=-170, max_position=170, max_velocity=140),
            "A6": JointLimit(min_position=-120, max_position=120, max_velocity=180),
            "A7": JointLimit(min_position=-175, max_position=175, max_velocity=180),
        },
    ),
    "AR7607": LBRSpecification(
        name="iiwa14",
        kuka_id="AR7607",
        mass=29.9,
        joint_limits={
            "A1": JointLimit(min_position=-170, max_position=170, max_velocity=85),
            "A2": JointLimit(min_position=-120, max_position=120, max_velocity=85),
            "A3": JointLimit(min_position=-170, max_position=170, max_velocity=100),
            "A4": JointLimit(min_position=-120, max_position=120, max_velocity=75),
            "A5": JointLimit(min_position=-170, max_position=170, max_velocity=130),
            "A6": JointLimit(min_position=-120, max_position=120, max_velocity=135),
            "A7": JointLimit(min_position=-175, max_position=175, max_velocity=135),
        },
    ),
    "AR16388": LBRSpecification(
        name="med7",
        kuka_id="AR16388",
        mass=25.5,
        joint_limits={
            "A1": JointLimit(min_position=-170, max_position=170, max_velocity=98),
            "A2": JointLimit(min_position=-120, max_position=120, max_velocity=98),
            "A3": JointLimit(min_position=-170, max_position=170, max_velocity=100),
            "A4": JointLimit(min_position=-120, max_position=120, max_velocity=130),
            "A5": JointLimit(min_position=-170, max_position=170, max_velocity=140),
            "A6": JointLimit(min_position=-120, max_position=120, max_velocity=180),
            "A7": JointLimit(min_position=-175, max_position=175, max_velocity=180),
        },
    ),
    "AR16387": LBRSpecification(
        name="med14",
        kuka_id="AR16387",
        mass=32.3,
        joint_limits={
            "A1": JointLimit(min_position=-170, max_position=170, max_velocity=85),
            "A2": JointLimit(min_position=-120, max_position=120, max_velocity=85),
            "A3": JointLimit(min_position=-170, max_position=170, max_velocity=100),
            "A4": JointLimit(min_position=-120, max_position=120, max_velocity=75),
            "A5": JointLimit(min_position=-170, max_position=170, max_velocity=130),
            "A6": JointLimit(min_position=-120, max_position=120, max_velocity=135),
            "A7": JointLimit(min_position=-175, max_position=175, max_velocity=135),
        },
    ),
}
