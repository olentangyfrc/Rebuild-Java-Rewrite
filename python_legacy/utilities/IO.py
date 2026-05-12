import math

from choreo.trajectory import SwerveSample, SwerveTrajectory
from phoenix6.status_signal import StatusSignal
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState

from phoenix6 import units
from wpimath.geometry import Pose3d, Rotation3d, Translation3d
from utilities.IO_helpers import (
    IO,
    PublisherSpecifications,
    auto_log,
    DeletionPolicy,
    DeletionType,
)


RATIO = 0.09588832710352932  # Gear Ratio for SwerveModule

FMS_ATTACHED = True


@auto_log
class SwerveIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        groups={
            ("*current*", "*voltage*", "*power*"): "power readings",
            "*supplier": "motor data",
        },
        container="Modules",
        ignore_fields={
            "angular_velocity_supplier",
            "steer_voltage_supplier",
            "drive_voltage_supplier",
            "velocity_supplier",
            "position_supplier",
            "*current*",
            "*voltage*",
            "*power*",
            "*supplier*",
            "*error*",
            "*angle*",
            "*angle_error*",
        },
        modifiers={
            "angle_supplier": lambda _: _ * 360 % 360,
            "angle_error": lambda _: 360 - _ if _ > 180 else _,
            "velocity_supplier": lambda _: _ * RATIO,
        },
        insert_class_name={"all"},
        # deletions={
        #     ("*current*", "*voltage*", "*power*"): DeletionPolicy(
        #         lambda: FMS_ATTACHED, DeletionType.ON_TRUE
        #     )
        # },
    )

    position_supplier: StatusSignal[float]
    velocity_supplier: StatusSignal[float]

    steer_motor_voltage_supplier: StatusSignal[float]
    drive_motor_voltage_supplier: StatusSignal[float]

    drive_supply_voltage_supplier: StatusSignal[units.volt]
    steer_supply_voltage_supplier: StatusSignal[units.volt]

    angle_supplier: StatusSignal[float]
    angular_velocity_supplier: StatusSignal[float]

    drive_stator_current_supplier: StatusSignal[float]
    drive_supply_current_supplier: StatusSignal[float]

    steer_stator_current_supplier: StatusSignal[float]
    steer_supply_current_supplier: StatusSignal[float]

    absolute_angle_supplier: StatusSignal[units.rotation]

    def __init__(self, name: str) -> None:
        self.velocity_error = 0.0
        self.angle_error = 0.0
        self.desired_speed = 0.0
        self.desired_angle = 0.0
        super().__init__(name)


@auto_log
class DrivetrainIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        groups={
            ("sample*", "auto*"): "auto",
            ("target*", "odometery*", "*future_pose*"): "pose info/other",
            "stable*": "pose info/rotation readings",
            "pose*": "pose info",
            ("applied*", "actual*", "commanded*"): "speeds",
            ("*module*"): "module data",
        },
        ignore_fields={
            "*signal",
            "last*",
            "gyro_yaw_supplier",
            "gyro_angular_speed_supplier",
            "*power*",
            "*current**applied*",
            "desired_module_statesauto_px",
            "auto_py",
            "auto_pomega",
            "tuning_sendables_sent",
            "sample_speed",
            "field_relative",
            "*future pose",
            "*odometry_pose",
            "actual_speed_magnitude",
            "applied_speed",
            "tuning_sendables_sent",
        },
        conditions={
            (
                "commanded_speed",
                "tuning*",
            ): DriverStation.isTeleop,  # applied should be same as sample,
        },
        high_resolution={"pose", "sample pose", "*module_states"},
        modifiers={
            "target_rotation": lambda _: _.degrees() % 360,
            "stable_rotation": lambda _: _.degrees() % 360,
        },
        insert_class_name={"state"},
        # deletions={
        #     ("auto*", "sample*"): DeletionPolicy(
        #         DriverStation.isAutonomous, DeletionType.TRUE_TO_FALSE
        #     ),
        #     (
        #         "*current*",
        #         "*voltage*",
        #         "*power*",
        #         "applied_speed",
        #         "field_relative",
        #         "gyro_velocity",
        #         "desired_module_states",
        #         "future_pose",
        #     ): DeletionPolicy(lambda: FMS_ATTACHED, DeletionType.ON_TRUE),
        # },
    )

    gyro_yaw_supplier: StatusSignal[float]
    gyro_angular_speed_supplier: StatusSignal[float]

    def __init__(self) -> None:
        self.state = "None"

        self.applied_speed = ChassisSpeeds()
        self.field_relative = True

        # The speed that the robot is actually driving at
        self.actual_speed = ChassisSpeeds()
        self.actual_speed_magnitude = 0.0

        self.pose_str = ""
        self.odometery_pose = Pose2d()

        self.target_pose = Pose2d()
        self.target_rotation = Rotation2d()

        self.desired_module_states = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )

        self.auto_px = 0.0
        self.auto_py = 0.0
        self.auto_pomega = 0.0

        self._sample_pose = Pose2d()
        self._sample_speed = ChassisSpeeds()

        # Stuff I kind of stuffed here to keep drivetrain.py cleaner

        self.last_module_positions = (
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
        )

        self.last_auton_trajectory = SwerveTrajectory("", [], [], [])
        self.tuning_senables_sent = False

        super().__init__()

    @property
    def sample(self) -> AttributeError:
        """Write only property. Sets a SwerveSample object to log multiple values from with a simpler syntax"""
        raise AttributeError("Sample property is write only. Do not access it.")

    @sample.setter
    def sample(self, sample: SwerveSample) -> None:
        self._sample_pose = sample.get_pose()
        self._sample_speed = sample.get_chassis_speeds()


@auto_log
class RobotIO(IO):
    SPECIFICATIONS = PublisherSpecifications(insert_class_name={"state", "voltage"})

    def __init__(self) -> None:
        self.state = ""
        self.time = 0.0
        self.voltage = 0.0

        self.shift = ""
        self.battery_saver_enabled = False
        self.energy_profile = ""
        self.aligned = False
        super().__init__()


@auto_log
class VisionIO(IO):
    SPECIFICATIONS = PublisherSpecifications(ignore_fields={"fused_pose_str"})

    most_recent_mt1_pose_measurements: tuple[
        Pose2d, ...
    ]  # a tuple of the most recent measures for each limelight in order

    most_recent_mt2_pose_measurements: tuple[
        Pose2d, ...
    ]  # a tuple of the most recent measures for each limelight in order

    def __init__(self) -> None:
        self.fused_pose_str = ""


@auto_log
class ShooterIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        high_resolution={
            "get_flywheel_speed_filtered",
            "feeder_current_supplier",
            "feeder_torque_current_supplier",
        },
        ignore_fields={
            "right_top_supply_voltage_supplier",
            "*velocity_supplier*",
            "tuning_sendables_sent",
            "*power*",
            # "clamped_rpm",
            "tuning_sendables_sent",
            "hood_zeroed",
            "*voltage*",
            "*stator_current*",
            "*supply_current*shooter_voltage",
            "*hood_supply*",
            "hood_position_supplier",
            "clamped_rpm",
            "*current*",
        },
        groups={
            "*hood*": "hood",
            ("*current*", "*voltage*", "*power*"): "power readings",
            ("*flywheel*", "*shooter*", "*rpm*"): "shooter",
            ("*feeder*", "*indexer"): "indexer",
        },
        insert_class_name={"state"},
        # deletions={
        #     (
        #         "*voltage*",
        #         "*power*",
        #         "clamped_rpm",
        #         "tuning_sendables_sent",
        #         "hood_zeroed",
        #     ): DeletionPolicy(lambda: FMS_ATTACHED, DeletionType.ON_TRUE),
        # },
    )

    left_bottom_velocity_supplier: StatusSignal[units.rotations_per_second]
    left_top_velocity_supplier: StatusSignal[units.rotations_per_second]
    right_bottom_velocity_supplier: StatusSignal[units.rotations_per_second]
    right_top_velocity_supplier: StatusSignal[units.rotations_per_second]

    left_bottom_supply_current_supplier: StatusSignal[units.ampere]
    left_top_supply_current_supplier: StatusSignal[units.ampere]
    right_bottom_supply_current_supplier: StatusSignal[units.ampere]
    right_top_supply_current_supplier: StatusSignal[units.ampere]

    left_bottom_stator_current_supplier: StatusSignal[units.ampere]
    left_top_stator_current_supplier: StatusSignal[units.ampere]
    right_bottom_stator_current_supplier: StatusSignal[units.ampere]
    right_top_stator_current_supplier: StatusSignal[units.ampere]

    left_bottom_supply_voltage_supplier: StatusSignal[units.volt]
    left_top_supply_voltage_supplier: StatusSignal[units.volt]
    right_bottom_supply_voltage_supplier: StatusSignal[units.volt]
    right_top_supply_voltage_supplier: StatusSignal[units.volt]

    feeder_supply_current_supplier: StatusSignal[units.ampere]
    feeder_stator_current_supplier: StatusSignal[units.ampere]
    feeder_supply_voltage_supplier: StatusSignal[units.volt]
    feeder_torque_current_supplier: StatusSignal[units.ampere]

    hood_position_supplier: StatusSignal[units.rotation]

    hood_supply_current_supplier: StatusSignal[units.ampere]
    hood_supply_voltage_supplier: StatusSignal[units.volt]

    indexer1_rps_supplier: StatusSignal[units.rotations_per_second]
    indexer2_rps_supplier: StatusSignal[units.rotations_per_second]

    def __init__(self) -> None:
        self.target_rpm = 0.0
        self.clamped_rpm = 0.0
        self.shooter_voltage = 0.0
        self.target_indexer1_rps = 0.0
        self.target_indexer2_rps = 0.0

        self.target_hood_angle = math.radians(20)
        self.hood_voltage = 0.0

        self.state = "IDLE"

        self.tuning_sendables_sent = False
        self.hood_zeroed = False
        super().__init__()


@auto_log
class SerializerIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        insert_class_name={"state", "voltage"},
        groups={("*current*", "*voltage*", "*power*"): "power readings"},
        ignore_fields={"*current*", "*voltage*", "*power*"},
        # deletions={
        #     ("*current*", "*voltage*", "*power*"): DeletionPolicy(
        #         DriverStation.isFMSAttached, DeletionType.ON_TRUE
        #     )
        # },
    )

    serializer_supply_voltage_supplier: StatusSignal[units.volt]
    serializer_supply_current_supplier: StatusSignal[units.ampere]
    serializer_stator_current_supplier: StatusSignal[units.ampere]
    serializer_rps_supplier: StatusSignal[units.rotations_per_second]

    def __init__(self) -> None:
        self.target_rps = 0.0
        self.state = "IDLE"
        super().__init__()


@auto_log
class VisualizerIO(IO):
    def __init__(self) -> None:
        self.component_poses = [Pose3d(), Pose3d()]
        super().__init__()

    def update(self, hood_angle_rad: float, intake_angle_rad: float):
        # Index 0: Hood (model_0.glb)
        hood_pose = Pose3d(
            Translation3d(0.1, 0.0, 0.5), Rotation3d(0, -(-hood_angle_rad + 0.25), 0)
        )

        # Index 1: Intake (model_1.glb)
        intake_pose = Pose3d(
            Translation3d(0, 0.0, 0), Rotation3d(0, -intake_angle_rad, 0)
        )

        self.component_poses = [hood_pose, intake_pose]


@auto_log
class IntakeIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        ignore_fields={
            "pivot_position_supplier",
            "pivot_velocity_supplier",
            "*current*",
            "*voltage*",
            "*power*",
            "stalled",
            "pivot_voltage",
            "*roller_velocity_supplier",
            "tuning_sendables_sent",
        },
        groups={("*current*", "*voltage*", "*power*"): "power readings"},
    )

    pivot_position_supplier: StatusSignal[units.rotation]
    pivot_velocity_supplier: StatusSignal[units.rotation]

    pivot_stator_current_supplier: StatusSignal[units.ampere]
    pivot_supply_current_supplier: StatusSignal[units.ampere]
    pivot_supply_voltage_supplier: StatusSignal[units.ampere]

    right_intake_roller_supply_current_supplier: StatusSignal[units.ampere]
    right_intake_roller_stator_current_supplier: StatusSignal[units.ampere]

    right_intake_roller_velocity_supplier: StatusSignal[float]

    left_intake_roller_supply_current_supplier: StatusSignal[units.ampere]
    left_intake_roller_stator_current_supplier: StatusSignal[units.ampere]

    left_intake_roller_velocity_supplier: StatusSignal[float]

    pivot_torque_current_supplier: StatusSignal[float]

    def __init__(self) -> None:
        self.state = "DEPLOYED"

        self.target_intake_roller_velocity = 0.0
        self.pivot_voltage = 0.0

        self.target_pivot_angle = 0.0
        self.tuning_sendables_sent = False
        super().__init__()


@auto_log
class ClimberIO(IO):
    SPECIFICATIONS = PublisherSpecifications(insert_class_name={"state", "voltage"})

    def __init__(self) -> None:
        self.target_setpoint = 0.0
        self.state = "IDLE"
        self.voltage_output = 0.0
        super().__init__()
