from collections.abc import Callable
import math
from logging import Logger

import wpimath.units as units
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import tunable
from phoenix6.hardware import Pigeon2
from phoenix6.configs import CurrentLimitsConfigs
from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard, Timer
from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard, Timer
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpiutil import Sendable, SendableBuilder

from components.modules.module import Module
from utilities.configs import DrivetrainConfig
from utilities.elasticlib import Notification, NotificationLevel, NotificationManager
from utilities.helpers import (
    DriveSignal,
    get_angle_between_poses,
    get_struct_string,
    inputModulus,
    within_pose_tolerance,
    within_rotation_tolerance,
)
from utilities.states import DrivetrainStates
from utilities.IO import DrivetrainIO
from utilities.vision_utils import VisionMeasurement

from utilities.rebuilt_helpers import GamePositions


class Drivetrain:
    logger: Logger

    kA = 0
    kAlpha = 0

    # Tune These once and call it a day
    pitch_offset = -0.52734375
    roll_offset = +9.66796875

    manual_tuning_mode = tunable(False)
    operator_lock = tunable(False)

    enable_vision = tunable(True)
    rot_stdev = 1.5

    MT2_ANGULAR_VELOCITY_THRESHOLD = 20
    MT1_ROTATION_DIFFERENCE_THRESHOLD = 20

    general_stdev_multiplier = 2.1
    extra_mt1_stdev_multiplier = 2.5
    mt1_with_mt2_stdev_multiplier = (
        7.5  # increase mt1 even more if we have valid mt2 measurement
    )

    class DrivetrainSendable(Sendable):
        def __init__(
            self,
            modules: tuple[Module, Module, Module, Module],
            gyro: Pigeon2,
            angular_offset: units.degrees = 0,
        ) -> None:
            self.modules = modules
            self.gyro = gyro
            self.angular_offset = angular_offset
            super().__init__()

        def initSendable(self, builder: SendableBuilder) -> None:
            builder.setSmartDashboardType("SwerveDrive")
            builder.addDoubleProperty(
                "Robot Angle",
                lambda: math.radians(self.gyro.get_yaw().value - self.angular_offset),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Front Left Velocity",
                lambda: self.modules[0].get_drive_velocity(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Front Left Angle",
                lambda: self.modules[0].get_angle(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Front Right Velocity",
                lambda: self.modules[1].get_drive_velocity(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Front Right Angle",
                lambda: self.modules[1].get_angle(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Back Left Velocity",
                lambda: self.modules[2].get_drive_velocity(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Back Left Angle",
                lambda: self.modules[2].get_angle(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Back Right Velocity",
                lambda: self.modules[3].get_drive_velocity(),
                lambda _: None,
            )
            builder.addDoubleProperty(
                "Back Right Angle",
                lambda: self.modules[3].get_angle(),
                lambda _: None,
            )

    def __init__(self, config: DrivetrainConfig) -> None:
        self.test_on_blocks = False
        self.current_sample = None
        self.auto_speed_limit = None
        self.config = config
        self.io = DrivetrainIO()

        self.modules = (
            self._build_module("Front Left", 0),
            self._build_module("Front Right", 1),
            self._build_module("Back Left", 2),
            self._build_module("Back Right", 3),
        )

        self.gyro = Pigeon2(self.config.gyro_id, self.config.CANbus)

        self.io.gyro_yaw_supplier = self.gyro.get_yaw()
        self.io.gyro_angular_speed_supplier = self.gyro.get_angular_velocity_z_device()

        self.io.gyro_yaw_supplier.set_update_frequency(250)
        self.io.gyro_angular_speed_supplier.set_update_frequency(100)
        self.gyro.set_yaw(0)

        fl_loc = Translation2d(self.config.wheel_base / 2, self.config.track_width / 2)
        fr_loc = Translation2d(self.config.wheel_base / 2, -self.config.track_width / 2)
        bl_loc = Translation2d(-self.config.wheel_base / 2, self.config.track_width / 2)
        br_loc = Translation2d(
            -self.config.wheel_base / 2, -self.config.track_width / 2
        )

        self.kinematics = SwerveDrive4Kinematics(fl_loc, fr_loc, bl_loc, br_loc)

        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_gyro_rotation(),
            self.get_module_positions(),
            Pose2d(),
        )
        self.pose_estimator.setVisionMeasurementStdDevs((0.1, 0.1, 0.1))

        self.odometery = SwerveDrive4Odometry(
            self.kinematics,
            self.get_gyro_rotation(),
            self.get_module_positions(),
            Pose2d(),
        )

        self.rotation_pid = PIDController(
            self.config.rotation_pid.p,
            self.config.rotation_pid.i,
            self.config.rotation_pid.d,
        )
        self.rotation_pid.setTolerance(math.radians(2))

        self.translation_pid = PIDController(
            self.config.translation_pid.p,
            self.config.translation_pid.i,
            self.config.translation_pid.d,
        )
        self.x_pid = PIDController(
            self.config.auto_pid.p, self.config.auto_pid.i, self.config.auto_pid.d
        )
        self.y_pid = PIDController(
            self.config.auto_pid.p, self.config.auto_pid.i, self.config.auto_pid.d
        )
        self.rotation_pid.enableContinuousInput(0, math.tau)

        self.fast_snap_pid = PIDController(
            self.config.fast_snap_pid.p,
            self.config.fast_snap_pid.i,
            self.config.fast_snap_pid.d,
        )
        self.fast_snap_pid.enableContinuousInput(0, math.tau)
        self.fast_snap_pid.setTolerance(math.radians(1))

        self.snake_pid = PIDController(
            self.config.snake_pid.p, self.config.snake_pid.i, self.config.snake_pid.d
        )
        self.snake_pid.enableContinuousInput(0, math.tau)
        self.snake_pid.setTolerance(math.radians(2.5))

        self.timer = Timer()
        self.drivetrain_field = Field2d()

        self.last_discretization_timestamp = 0.0
        self.last_rotational_timestamp = 0.0
        self.last_translational_timestamp = 0.0
        self.last_automatic_operator_lock_timestamp = 0.0

        self.rotation_limit = None
        self.translation_limit = None
        self.state = None
        self._signal = DriveSignal()

        self.heading_assist = False
        self.y_assist = False
        self.x_assist = False

        self.use_mt1_only = False

        self.assist_target_x = 0
        self.assist_target_y = 0
        self.assist_target_rot = 0

        self.aligned_callable: Callable[[], bool] = lambda: False

        self._set_up_logging()
        self._set_up_notifications()

    @property
    def target_rotation(self) -> Rotation2d:
        return self.io.target_rotation

    @target_rotation.setter
    def target_rotation(self, rotation: Rotation2d) -> None:
        if not isinstance(rotation, Rotation2d):
            raise ValueError("Target rotation must be a Rotation2d object")

        self.io.target_rotation = rotation

    @property
    def target_pose(self) -> Pose2d:
        return self.io.target_pose

    @target_pose.setter
    def target_pose(self, pose: Pose2d) -> None:
        if not isinstance(pose, Pose2d):
            raise ValueError("Target pose must be a Pose2d object")

        self.io.target_pose = pose

    @property
    def signal(self) -> DriveSignal:
        return self._signal

    @signal.setter
    def signal(self, signal: DriveSignal) -> None:
        if not isinstance(signal, DriveSignal):
            raise ValueError("Signal can only be set to type DriveSignal")

        self.io.field_relative = signal.get_field_relative()

        self._signal = signal

    def get_gyro_rotation(self) -> Rotation2d:
        if self.test_on_blocks and self.current_sample and DriverStation.isAutonomous():
            return Rotation2d(self.current_sample.heading)
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value)

    def get_gyro_velocity(self) -> units.degrees_per_second:
        if self.test_on_blocks and self.current_sample and DriverStation.isAutonomous():
            return math.degrees(self.current_sample.omega)
        return self.gyro.get_angular_velocity_z_device().value

    def get_pitch(self) -> float:
        return self.gyro.get_pitch().value - self.pitch_offset

    def get_roll(self) -> float:
        return self.gyro.get_roll().value - self.roll_offset

    def get_pose(self) -> Pose2d:
        if self.test_on_blocks and self.current_sample and DriverStation.isAutonomous():
            return Pose2d(
                self.current_sample.x,
                self.current_sample.y,
                self.get_gyro_rotation(),
            )
        estimator_pose = self.pose_estimator.getEstimatedPosition()
        self.io.odometery_pose = self.odometery.getPose()

        self.io.pose_str = get_struct_string(estimator_pose)

        return estimator_pose

    def get_future_pose(self, dt: float = 0.05) -> Pose2d:
        future_pose = self.get_pose().exp(
            Twist2d(
                self.io.applied_speed.vx * dt,
                self.io.applied_speed.vy * dt,
                self.io.applied_speed.omega * dt,
            )
        )

        return future_pose

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return [module.get_position() for module in self.modules]  # type: ignore

    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return [module.get_state() for module in self.modules]  # type: ignore

    def get_distance_to_hub(self) -> units.meters:
        return (
            self.get_pose()
            .translation()
            .distance(GamePositions.HUB.value.translation())
        )

    def get_angle_to_hub(self) -> units.radians:
        return (
            get_angle_between_poses(self.get_pose(), GamePositions.HUB.value) + math.pi
        )

    def get_adjusted_angle_to_hub(self) -> units.radians:
        field_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            self.io.actual_speed, self.get_gyro_rotation()
        )

        adjusted_pose = Pose2d(
            self.get_pose().X() + field_speeds.vx * 0.02,
            self.get_pose().Y() + field_speeds.vy * 0.02,
            self.get_pose().rotation().radians() + self.io.actual_speed.omega,
        )
        return get_angle_between_poses(adjusted_pose, GamePositions.HUB.value) + math.pi

    def get_adjusted_distance_to_hub(self) -> units.meters:
        field_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            self.io.actual_speed, self.get_gyro_rotation()
        )

        d = self.get_distance_to_hub()
        omega = self.get_angle_to_hub()
        vx = field_speeds.vx * 0.02
        vy = field_speeds.vy * 0.02

        radicand = (
            d**2 + 2 * d * (vx * math.cos(omega) + vy * math.sin(omega)) + vx**2 + vy**2
        )

        return math.sqrt(radicand)

    def reset_pose(self, new: Pose2d) -> None:
        self.pose_estimator.resetTranslation(new.translation())
        self.gyro.set_yaw(new.rotation().degrees())
        # self.pose_estimator.resetPose(new)

    def enable_motion_limiting(
        self, translation_limit: float = 0.15, rotation_limit: float = 0.15
    ) -> None:
        # Prevents spam when holding button
        if not (
            self.translation_limit == translation_limit
            or self.rotation_limit == rotation_limit
        ):
            NotificationManager.add_unconditional_notification(
                Notification(
                    title="Motion limiting enabled",
                    description=f"Translation speed will be limited to {self.config.max_translation_speed * translation_limit} mps and rotation speed will be limited to {self.config.max_rotation_speed * rotation_limit} mps",
                )
            )
        self.translation_limit = translation_limit
        self.rotation_limit = rotation_limit

    def disable_motion_limiting(self) -> None:
        # NotificationManager.add_unconditional_notification(
        #     Notification(
        #         title="Motion limiting is disabled",
        #         description=f"Translation speed is reset to {self.config.max_translation_speed} mps and rotation speed is reset to {self.config.max_rotation_speed} mps",
        #     )
        # )
        self.translation_limit = None
        self.rotation_limit = None

    def enable_mt1_only(self) -> None:
        self.use_mt1_only = True

    def disable_mt1_only(self) -> None:
        self.use_mt1_only = False

    def go_to_rotation(self, rotation: Rotation2d) -> None:
        self.target_rotation = rotation
        self.state = DrivetrainStates.ACTIVE_SNAP

    def go_to_pose(self, pose: Pose2d) -> None:
        self.target_pose = pose
        self.state = DrivetrainStates.LINE_UP

    def point_to_pose(self, pose: Pose2d) -> None:
        self.target_pose = pose
        self.state = DrivetrainStates.POINT_AT_POSE

    def apply_current_limits(self, current_limits: CurrentLimitsConfigs) -> None:
        for module in self.modules:
            module.apply_current_limit(current_limits)

    def follow_trajectory(
        self,
        sample: SwerveSample,
        traj: SwerveTrajectory,
        is_reversed: bool = False,
        is_heading_locked: bool = False,
        test_on_blocks: bool = False,
        auto_speed_limit: float | None = None,
    ) -> None:
        self.test_on_blocks = test_on_blocks
        self.auto_speed_limit = auto_speed_limit

        if sample is None:
            NotificationManager.add_unconditional_notification(
                Notification(
                    level=NotificationLevel.ERROR,
                    title="Invalid sample provided",
                )
            )
            self.state = DrivetrainStates.LOCK
            return

        self.io.sample = sample
        self.current_sample = sample

        # To minimize the amount of trajectories we generate
        if traj != self.io.last_auton_trajectory:
            self.drivetrain_field.getObject("traj").setPoses(traj.get_poses())
            self.io.last_auton_trajectory = traj

        pose = self.get_pose()

        gain_multiplier = 1.0

        self.io.auto_px = self.x_pid.calculate(pose.X(), sample.x) * gain_multiplier
        self.io.auto_py = self.y_pid.calculate(pose.Y(), sample.y) * gain_multiplier

        original_p = self.rotation_pid.getP()
        if is_heading_locked:
            self.rotation_pid.setP(0.8)

        self.io.auto_pomega = self.fast_snap_pid.calculate(
            self.get_gyro_rotation().radians(), sample.heading
        )

        if is_heading_locked:
            self.rotation_pid.setP(original_p)

        speed = ChassisSpeeds(
            sample.ax * self.kA + sample.vx + self.io.auto_px,
            sample.ay * self.kA + sample.vy + self.io.auto_py,
            sample.omega + self.kAlpha * sample.alpha + self.io.auto_pomega,
        )

        self.signal = DriveSignal(speed)

    def is_aligned(
        self,
        translation_tolerance: units.meters | None,
        rotation_tolerance: units.degrees,
        y_only: bool = False,
        rotation_only: bool = False,
    ) -> bool:
        if translation_tolerance is None and not rotation_only:
            raise ValueError(
                "Please pass translation_tolerance if not lining up rotation_tolerance"
            )

        if self.state not in [
            DrivetrainStates.ACTIVE_SNAP,
            DrivetrainStates.LINE_UP,
            DrivetrainStates.POINT_AT_POSE,
            DrivetrainStates.SNAKE,
            DrivetrainStates.LOCK,
        ]:
            return False

        if (
            self.state
            in [
                DrivetrainStates.ACTIVE_SNAP,
                DrivetrainStates.SNAKE,
                DrivetrainStates.LOCK,
                DrivetrainStates.POINT_AT_POSE,
            ]
            and rotation_only
        ):
            return within_rotation_tolerance(
                self.get_pose().rotation(), self.target_rotation, rotation_tolerance
            )

        if y_only and self.state is DrivetrainStates.LINE_UP:
            pose = self.get_pose()
            y_aligned = abs(self.target_pose.Y() - pose.Y()) <= translation_tolerance  # type: ignore
            rot_aligned = within_rotation_tolerance(
                pose.rotation(), self.target_pose.rotation(), rotation_tolerance
            )
            return y_aligned and rot_aligned

        return within_pose_tolerance(
            self.target_pose,
            self.get_pose(),
            translation_tolerance,
            rotation_tolerance,  # type: ignore
        )

    def is_stationary(self) -> bool:
        for module in self.modules:
            if not math.isclose(module.get_magnitude(), 0, abs_tol=0.1):
                return False

        return bool(math.isclose(self.signal.get_magnitude(), 0, abs_tol=0.1))

    def is_motion_limited(self) -> bool:
        return self.translation_limit is not None or self.rotation_limit is not None

    def lock(self) -> None:
        self.state = DrivetrainStates.LOCK

    def stop(self) -> None:
        self.state = None
        self.signal = DriveSignal(ChassisSpeeds())
        self.target_pose = Pose2d()
        self.target_rotation = Rotation2d()
        self.test_on_blocks = False
        self.current_sample = None
        self.auto_speed_limit = None

    def activate_assists(
        self,
        x_assist: bool = False,
        y_assist: bool = False,
        heading_assist: bool = False,
        target_x: float = 0.0,
        target_y: float = 0.0,
        target_rot: float = 0.0,
    ) -> None:
        self.x_assist = x_assist
        self.y_assist = y_assist
        self.heading_assist = heading_assist

        self.target_pose = Pose2d(
            target_x if x_assist else self.get_pose().X(),
            target_y if y_assist else self.get_pose().Y(),
            math.radians(target_rot)
            if heading_assist
            else self.get_pose().rotation().radians(),
        )

        self.state = DrivetrainStates.DRIVER_ASSIST

    def drive(self, signal: DriveSignal) -> None:
        if self.translation_limit is not None:
            signal.speed.vx *= self.translation_limit
            signal.speed.vy *= self.translation_limit
        if self.rotation_limit is not None:
            signal.speed.omega *= self.rotation_limit

        if signal.get_field_relative():
            signal.speed = ChassisSpeeds.fromFieldRelativeSpeeds(
                signal.speed, self.get_gyro_rotation()
            )

        if self.config.enable_discretization:
            signal.speed = ChassisSpeeds.discretize(
                signal.speed,
                self.timer.getFPGATimestamp() - self.last_discretization_timestamp,
            )
            self.last_discretization_timestamp = self.timer.getFPGATimestamp()

        self.io.applied_speed = signal.speed

        max_trans_speed = (
            self.auto_speed_limit
            if DriverStation.isAutonomous() and self.auto_speed_limit is not None
            else self.config.max_translation_speed
        )

        target_states = self.kinematics.desaturateWheelSpeeds(
            self.kinematics.toSwerveModuleStates(signal.speed),
            signal.speed,
            self.config.swerve_config.max_module_speed,
            max_trans_speed,
            self.config.max_rotation_speed,
        )

        self.io.desired_module_states = target_states

        for state, module in zip(target_states, self.modules):
            module.set_desired_state(state)

    def snake(self) -> None:
        """Intake always pointing forwards mode"""
        vx = self.signal.speed.vx
        vy = self.signal.speed.vy

        if math.isclose(self.signal.get_magnitude(), 0, abs_tol=0.01):
            self.target_rotation = self.get_gyro_rotation()
        else:
            self.target_rotation = Rotation2d(
                math.atan2(vy, vx)
            )  # Simple discretization algorithm
        self.state = DrivetrainStates.SNAKE

    def update_odometery(self) -> None:
        self.odometery.update(self.get_gyro_rotation(), self.get_module_positions())
        self.pose_estimator.updateWithTime(
            self.timer.getFPGATimestamp(),
            self.get_gyro_rotation(),
            self.get_module_positions(),
        )

        self.io.actual_speed = self.kinematics.toChassisSpeeds(self.get_module_states())
        self.io.actual_speed_magnitude = math.hypot(
            self.io.actual_speed.vx, self.io.actual_speed.vy
        )

        self.io.last_module_positions = self.get_module_positions()

        self.drivetrain_field.setRobotPose(self.get_pose())

        self.drivetrain_field.getObject("Adjusted hub target").setPose(
            Pose2d(
                self.get_pose().X()
                + self.get_adjusted_distance_to_hub()
                * math.cos(self.get_adjusted_angle_to_hub() - math.pi),
                self.get_pose().Y()
                + self.get_adjusted_distance_to_hub()
                * math.sin(self.get_adjusted_angle_to_hub() - math.pi),
                0,
            )
        )

        # self.drivetrain_field.getObject("Future Pose").setPose(self.get_future_pose())
        # self.drivetrain_field.getObject("Target Pose").setPose(
        # self.target_pose if self.target_pose != Pose2d() else Pose2d(100, 100, 0)
        # )

    def add_vision_measurement(
        self,
        mt2_measurements: list[VisionMeasurement],
        mt1_measurements: list[VisionMeasurement],
    ) -> None:
        has_mt2 = False
        gyro_vel = self.get_gyro_velocity()

        if not self.use_mt1_only:
            for mt2_measurement in mt2_measurements:
                rotation_modifier = (abs(gyro_vel)) * self.rot_stdev
                rotation_modifier = rotation_modifier if rotation_modifier != 0 else 1

                if (
                    abs(mt2_measurement.pose.translation().norm()) > 1e-6
                    and gyro_vel < self.MT2_ANGULAR_VELOCITY_THRESHOLD
                ):
                    self.pose_estimator.addVisionMeasurement(
                        mt2_measurement.pose,
                        mt2_measurement.timestamp,
                        (
                            mt2_measurement.xy_std
                            * rotation_modifier
                            * self.general_stdev_multiplier,
                            mt2_measurement.xy_std
                            * rotation_modifier
                            * self.general_stdev_multiplier,
                            mt2_measurement.rot_std,
                        ),
                    )

        if self.enable_vision and self.timer.get() > 0.3:
            for mt1_measurement in mt1_measurements:
                if (
                    abs(mt1_measurement.pose.translation().norm()) > 1e-6
                ):  # close to zero means no vision
                    mt1_timestamp_estimated_pose = self.pose_estimator.sampleAt(
                        mt1_measurement.timestamp
                    )
                    mt1_modfier = (
                        self.extra_mt1_stdev_multiplier * self.general_stdev_multiplier
                    )

                    if has_mt2 and not self.use_mt1_only:
                        mt1_modfier *= self.mt1_with_mt2_stdev_multiplier

                    if not mt1_timestamp_estimated_pose:
                        continue

                    if (
                        abs(
                            mt1_timestamp_estimated_pose.rotation().degrees()
                            - mt1_measurement.pose.rotation().degrees()
                        )
                        < self.MT1_ROTATION_DIFFERENCE_THRESHOLD
                    ):
                        self.pose_estimator.addVisionMeasurement(
                            mt1_measurement.pose,
                            mt1_measurement.timestamp,
                            (
                                mt1_measurement.xy_std * mt1_modfier,
                                mt1_measurement.xy_std * mt1_modfier,
                                mt1_measurement.rot_std,
                            ),
                        )

        # if (
        #     self.enable_vision
        #     and abs(measurement.pose.translation().norm()) > 1e-6
        #     and self.timer.get() > 0.3
        # ):
        #     modifier = (abs(self.get_gyro_velocity())) * self.rot_stdev
        #     modifier = modifier if modifier != 0 else 1
        #     self.pose_estimator.setVisionMeasurementStdDevs(
        #         (
        #             measurement.xy_std * modifier,
        #             measurement.xy_std * modifier,
        #             measurement.rot_std,
        #         )
        #     )
        #     self.pose_estimator.addVisionMeasurement(
        #         measurement.pose, measurement.timestamp
        #     )

    def execute(self) -> None:
        if not self.timer.isRunning():
            self.timer.start()

        self.update_odometery()

        if (
            self.manual_tuning_mode
            and self.operator_lock
            and self.io.tuning_senables_sent
        ):  # Last condition ensures you can actually access the PID controller
            for module in self.modules:
                module.manually_control()
            return

        if self.manual_tuning_mode and not self.io.tuning_senables_sent:
            self._send_tuning_sendables()

        # self._handle_passive_state_transitions()

        if self.operator_lock:
            self._lock_chassis()

        self._handle_state_logic()

        if self.state is DrivetrainStates.LOCK:
            return

        self.drive(self.signal)

    def _lock_chassis(self) -> None:
        if self.state != DrivetrainStates.LOCK:  # Ensures stop on original transition
            for module in self.modules:
                module.stop()
        self.state = DrivetrainStates.LOCK
        self.signal = DriveSignal(ChassisSpeeds(0, 0, 0))

    def _handle_passive_state_transitions(self) -> None:
        if not self.timer.isRunning():
            self.timer.start()

        if not math.isclose(self.signal.get_magnitude(), 0, abs_tol=0.1):
            self.last_translational_timestamp = self.timer.get()
        if not math.isclose(self.signal.speed.omega, 0, abs_tol=0.1):
            self.last_rotational_timestamp = self.timer.get()

        if (
            self.state
            not in [
                DrivetrainStates.ACTIVE_SNAP,
                DrivetrainStates.LINE_UP,
                DrivetrainStates.POINT_AT_POSE,
            ]
            and DriverStation.isTeleop()
            and not self.operator_lock
        ):
            # Passive state handling should only be in teleop, auton state handling is done in autonomous routine
            if (
                self.timer.get() - self.last_rotational_timestamp > 180
                and self.timer.get() - self.last_translational_timestamp > 180
                and self.config.automatically_lock
            ):
                if (
                    self.last_automatic_operator_lock_timestamp == 0
                    or self.timer.get() - self.last_automatic_operator_lock_timestamp
                    > 10
                ):  # gives time to unlock
                    self.operator_lock = True
                    self.last_automatic_operator_lock_timestamp = self.timer.get()
            elif (
                self.timer.get() - self.last_rotational_timestamp > 1.5
                and self.timer.get() - self.last_translational_timestamp > 1.5
                and self.config.automatically_lock
            ):
                self.state = DrivetrainStates.LOCK
            elif (
                self.timer.get() - self.last_rotational_timestamp > 0.04
                and self.config.enable_passive_snap
            ):
                self.state = DrivetrainStates.PASSIVE_SNAP
            else:
                self.state = None

    def _handle_state_logic(self) -> None:
        self.io.state = self.state.name if self.state is not None else "None"

        match self.state:
            case DrivetrainStates.PASSIVE_SNAP:
                self.target_rotation = self.get_pose().rotation()
                self.rotation_pid.setSetpoint(self.target_rotation.radians())
                self.signal.speed.omega = self.rotation_pid.calculate(
                    self.get_pose().rotation().radians()
                )
                # Stops jittering if not commanded to drive
                if math.isclose(self.signal.get_magnitude(), 0, abs_tol=0.1):
                    self.signal.speed.omega = 0
            case DrivetrainStates.ACTIVE_SNAP:
                self.aligned_callable = lambda: self.is_aligned(
                    translation_tolerance=None, rotation_tolerance=2, rotation_only=True
                )

                if self.manual_tuning_mode:
                    wrapped_input = inputModulus(
                        self.rotation_pid.getSetpoint(), 0, math.tau
                    )
                    self.target_rotation = Rotation2d(wrapped_input)

                self.rotation_pid.setSetpoint(self.target_rotation.radians())
                self.signal.speed.omega = self.rotation_pid.calculate(
                    self.get_pose().rotation().radians()
                )

                if self.aligned_callable() and not self.manual_tuning_mode:
                    self.state = None
                    self.target_rotation = Rotation2d()
            case DrivetrainStates.SNAKE:
                self.aligned_callable = lambda: self.is_aligned(
                    translation_tolerance=None,
                    rotation_tolerance=2.5,
                    rotation_only=True,
                )

                if self.manual_tuning_mode:
                    wrapped_input = inputModulus(
                        self.rotation_pid.getSetpoint(), 0, math.tau
                    )
                    self.target_rotation = Rotation2d(wrapped_input)

                self.snake_pid.setSetpoint(self.target_rotation.radians())
                self.signal.speed.omega = self.snake_pid.calculate(
                    self.get_gyro_rotation().radians()
                )

                if self.aligned_callable() and not self.manual_tuning_mode:
                    self.state = None
                    self.target_rotation = Rotation2d()
            case DrivetrainStates.POINT_AT_POSE:
                self.target_rotation = Rotation2d(
                    get_angle_between_poses(self.get_pose(), self.target_pose) + math.pi
                )
                self.fast_snap_pid.setSetpoint(self.target_rotation.radians())
                self.signal.speed.omega = self.fast_snap_pid.calculate(
                    self.get_pose().rotation().radians(),
                )
                if math.isclose(self.signal.speed.omega, 0, abs_tol=0.1):
                    self.signal.speed.omega = 0

                self.aligned_callable = lambda: self.is_aligned(
                    translation_tolerance=None, rotation_tolerance=2, rotation_only=True
                )
            case DrivetrainStates.LINE_UP:
                # if RobotBase.isReal():
                #     difference = (
                #         self.target_pose.translation() - self.get_pose().translation()
                #     )
                # else:
                self.aligned_callable = lambda: self.is_aligned(
                    translation_tolerance=0.07, rotation_tolerance=2
                )

                difference = (
                    self.get_pose().translation() - self.target_pose.translation()
                )

                speed = self.translation_pid.calculate(difference.norm(), 0)

                ratio = speed / difference.norm() if difference.norm() != 0 else 0
                self.signal.speed.vx = ratio * difference.X()
                self.signal.speed.vy = ratio * difference.Y()

                self.signal.speed.omega = self.rotation_pid.calculate(
                    self.get_pose().rotation().radians(),
                    self.target_pose.rotation().radians(),
                )

                if self.aligned_callable() and not self.manual_tuning_mode:
                    self.state = None
                    self.target_pose = Pose2d()
            case DrivetrainStates.LOCK:
                self.modules[0].set_desired_state(
                    SwerveModuleState(0, Rotation2d.fromDegrees(225))
                )
                self.modules[1].set_desired_state(
                    SwerveModuleState(0, Rotation2d.fromDegrees(135))
                )
                self.modules[2].set_desired_state(
                    SwerveModuleState(0, Rotation2d.fromDegrees(315))
                )
                self.modules[3].set_desired_state(
                    SwerveModuleState(0, Rotation2d.fromDegrees(225))
                )
            case DrivetrainStates.DRIVER_ASSIST:
                difference = (
                    self.get_pose().translation() - self.target_pose.translation()
                )

                speed = self.translation_pid.calculate(difference.norm(), 0)
                ratio = speed / difference.norm() if difference.norm() != 0 else 0

                if self.x_assist:
                    self.signal.speed.vx = ratio * difference.X()
                if self.y_assist:
                    self.signal.speed.vy = ratio * difference.Y()
                if self.heading_assist:
                    self.signal.speed.omega = self.rotation_pid.calculate(
                        self.get_gyro_rotation().radians(),
                        self.target_pose.rotation().radians(),
                    )

    def _set_up_logging(self) -> None:
        self.io.add_function(self.get_pose)

        self.io.add_function(self.get_distance_to_hub)

        self.io.add_function(self.get_module_positions)
        self.io.add_function(self.get_module_states)

        self.io.add_function(self.get_gyro_velocity)
        self.io.add_function(self.get_gyro_rotation)

        # self.io.add_function(self.get_adjusted_angle_to_hub, lambda _: math.degrees(_) - 180)
        # self.io.add_function(self.get_adjusted_distance_to_hub)

        self.io.add_function(self.get_angle_to_hub, lambda _: math.degrees(_) - 180)

        # self.io.add_function(lambda: self.get_adjusted_distance_to_hub() - self.get_distance_to_hub(), alternate_name="Delta D")
        # self.io.add_function(lambda: self.get_adjusted_angle_to_hub() - self.get_angle_to_hub(), modifier=math.degrees, alternate_name="Delta Angle")

        SmartDashboard.putData("Drivetrain Field", self.drivetrain_field)
        SmartDashboard.putData(
            "Swerve Drive", Drivetrain.DrivetrainSendable(self.modules, self.gyro, 0)
        )

        self.io.last_module_positions = self.get_module_positions()

    def _set_up_notifications(self) -> None:
        NotificationManager.add_conditional_notification(
            Notification(
                title="Passive snap enabled",
                description="robot will try to compensate for angular drift.",
            ),
            lambda: self.config.enable_passive_snap,
            False,
        )

        NotificationManager.add_conditional_notification(
            Notification(
                title="Chassis is discretized",
                description="robot is optimized to avoid translational error while rotating. "
                "May cause translational error in other areas.",
            ),
            lambda: self.config.enable_discretization,
            False,
        )

        NotificationManager.add_conditional_notification(
            Notification(
                title="Drivetrain is operator locked",
                description="This prevents the drivetrain from moving for safety reasons. Toggle switch on elastic to unlock.",
            ),
            lambda: self.operator_lock,
            cooldown=120,
        )

        NotificationManager.add_conditional_notification(
            Notification(
                title="Drivetrain is in tuning mode",
                description="Make sure to input rotation targets in radians and translation targets in meters",
            ),
            lambda: self.manual_tuning_mode,
            cooldown=300,
        )

        NotificationManager.add_conditional_notification(
            Notification(
                title="Drivetrain is shut down",
                description="Individual modules can now be controlled without interference",
            ),
            lambda: (
                self.operator_lock
                and self.manual_tuning_mode
                and self.io.tuning_senables_sent
            ),
            cooldown=300,
        )

    def _build_module(self, name: str, index: int) -> Module:
        return self.config.module_type(
            self.config.drive_motor_ids[index],
            self.config.steer_motor_ids[index],
            self.config.steer_encoder_ids[index],
            self.config.module_offsets[index],
            self.config.swerve_config,
            name,
            self.config.CANbus,
            self.config.drive_inverted[index],
            self.config.steer_inverted[index],
            self.config.encoder_inverted[index],
        )

    def _send_tuning_sendables(self) -> None:
        def add_to_smartdashboard(
            module: Module, attribute_name: str, display_name: str
        ) -> None:
            name = module.name

            try:
                SmartDashboard.putData(
                    f"{name} {display_name}", getattr(module, attribute_name)
                )
            except AttributeError as e:
                self.logger.warning(
                    f"Could not find attribute {attribute_name} in {module.__class__.__name__}.\nStack Trace: {e}"
                )
                return
            except TypeError as e:
                self.logger.warning(
                    f"SmartDashboard could not display type of {attribute_name} as a sendable.\nStack Trace: {e}"
                )

        SmartDashboard.putData("Rotation PID", self.rotation_pid)
        SmartDashboard.putData("Translation PID", self.translation_pid)
        SmartDashboard.putData("X PID", self.x_pid)
        SmartDashboard.putData("Y PID", self.y_pid)
        SmartDashboard.putData("Fast Snap PID", self.fast_snap_pid)

        # FF Tuning is a seperate process so not included here
        add_to_smartdashboard(self.modules[1], "steer_pid", "Steer PID")
        add_to_smartdashboard(self.modules[1], "drive_pid", "Drive PID")
        # add_to_smartdashboard(self.modules[0], "drive_motor", "Drive Motor")
        # add_to_smartdashboard(self.modules[0], "steer_motor", "Steer Motor")

        self.io.tuning_senables_sent = True
