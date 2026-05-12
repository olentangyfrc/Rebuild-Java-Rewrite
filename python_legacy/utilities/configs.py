from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from phoenix6 import CANBus
from phoenix6.configs import Slot0Configs
from wpimath import units
from wpimath.geometry import Transform3d

from utilities.helpers import (
    FREE_SPEED_LOOKUP,
    AnyAmountStr,
    FFConstants,
    MotorTypes,
    PIDConstants,
    ProfileConstants,
    CurrentConfigs,
)

# This deals with circular imports for type annotations
if TYPE_CHECKING:
    from components.modules.module import (
        Module,
    )


@dataclass
class SwerveConfig:
    drive_ratio: float
    steer_ratio: float
    steer_pid_constants: PIDConstants | Slot0Configs
    drive_pid_constants: PIDConstants | Slot0Configs
    ff_constants: FFConstants
    wheel_radius: units.meters
    drive_motor_type: MotorTypes

    @property
    def max_module_speed(self) -> float:
        return (
            FREE_SPEED_LOOKUP.get(self.drive_motor_type, 100)
            * self.drive_ratio
            * self.wheel_radius
            * math.tau
        )


@dataclass
class DrivetrainConfig:
    drive_motor_ids: tuple[int, int, int, int]
    steer_motor_ids: tuple[int, int, int, int]
    steer_encoder_ids: tuple[int, int, int, int]
    gyro_id: int
    module_offsets: tuple[float, float, float, float]
    drive_inverted: tuple[bool, bool, bool, bool]
    steer_inverted: tuple[bool, bool, bool, bool]
    encoder_inverted: tuple[bool, bool, bool, bool]
    max_translation_speed: units.meters_per_second
    max_rotation_speed: units.radians_per_second
    enable_discretization: bool
    enable_passive_snap: bool
    automatically_lock: bool
    wheel_base: units.meters
    track_width: units.meters
    rotation_pid: PIDConstants
    translation_pid: PIDConstants
    fast_snap_pid: PIDConstants
    snake_pid: PIDConstants
    auto_pid: PIDConstants
    module_type: type[Module]
    swerve_config: SwerveConfig
    CANbus: CANBus


@dataclass
class VisionConfig:
    camera_names: AnyAmountStr
    robot_to_camera_transformations: tuple[Transform3d, ...]  # For sim cameras
    max_frame_count: int | None = None
    time_delay: units.seconds = 1.0


@dataclass
class ShooterConfig:
    shooter_motor_ids: tuple[int, int, int, int]  # LMR
    indexer_id: tuple[int, int]
    hood_id: int
    hood_encoder_id: int
    # left_canrange: int
    # right_canrange: int
    shooter_gear_ratio: float
    hood_gear_ratio: float
    hood_offset: float
    shoot_ff_constants: FFConstants
    shoot_pid_constants: PIDConstants
    indexer1_ff_constants: FFConstants
    indexer1_pid_constants: PIDConstants
    indexer2_ff_constants: FFConstants
    indexer2_pid_constants: PIDConstants
    hood_ff_constants: FFConstants
    hood_pid_constants: PIDConstants
    CANbus: CANBus


@dataclass
class IntakeConfig:
    intake_roller_id: tuple[int, int]
    intake_roller_ff_constants: FFConstants
    intake_roller_pid_constants: PIDConstants

    pivot_motor_id: int
    pivot_encoder_id: int
    pivot_gear_ratio: float
    encoder_offset: float

    pivot_ff_constants: FFConstants
    pivot_pid_constants: PIDConstants
    pivot_profile_constants: ProfileConstants  # Usually a custom struct for max vel/acc

    CANbus: CANBus


@dataclass
class SerializerConfig:
    serializer_roller_id: int
    serializer_ff_constants: FFConstants
    serializer_pid_constants: PIDConstants
    CANBus: CANBus


@dataclass
class ClimberConfig:
    climber_motor_id: int
    climber_encoder_id: int
    climber_ff_constants: FFConstants
    climber_pid_constants: PIDConstants
    CANBus: CANBus
