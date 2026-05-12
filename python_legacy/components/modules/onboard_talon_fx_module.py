import math

import wpimath.units as units
from phoenix6 import CANBus
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.configs import (
    CANcoderConfiguration,
    CurrentLimitsConfigs,
    TalonFXConfiguration,
    Slot0Configs,
)
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    SensorDirectionValue,
    FeedbackSensorSourceValue,
    StaticFeedforwardSignValue,
)
from wpilib import RobotBase
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from components.modules.module import Module
from utilities.configs import SwerveConfig
from utilities.IO import SwerveIO


class OnboardTalonFxModule(Module):
    def __init__(
        self,
        drive_motor_id: int,
        steer_motor_id: int,
        steer_encoder_id: int,
        offset: float,
        config: SwerveConfig,
        name: str,
        canbus: CANBus,
        drive_inverted: bool,
        steer_inverted: bool,
        encoder_inverted: bool,
    ) -> None:
        super().__init__(
            drive_motor_id,
            steer_motor_id,
            steer_encoder_id,
            offset,
            config,
            name,
            canbus,
            drive_inverted,
            steer_inverted,
            encoder_inverted,
        )

        self.name = name
        self.config = config
        self.offset = offset
        self.io = SwerveIO(self.name)

        if not isinstance(
            self.config.drive_pid_constants, Slot0Configs
        ) or not isinstance(self.config.steer_pid_constants, Slot0Configs):
            raise ValueError(
                "Please pass in `Slot0Config` objects for config pid constants when using onboard control"
            )

        drive_config = TalonFXConfiguration()
        drive_config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        if drive_inverted:
            drive_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            drive_config.motor_output.inverted = (
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )

        drive_config.feedback.sensor_to_mechanism_ratio = 1 / (
            self.config.drive_ratio * math.tau * self.config.wheel_radius
        )

        drive_config.slot0 = self.config.drive_pid_constants

        self.drive_feed_forward = SimpleMotorFeedforwardMeters(
            self.config.ff_constants.kS,
            self.config.ff_constants.kV,
            self.config.ff_constants.kA,
        )

        self.drive_motor = TalonFX(drive_motor_id, canbus)
        self.drive_motor.clear_sticky_faults()
        self.drive_motor.configurator.apply(drive_config)

        encoder_config = CANcoderConfiguration()
        self.steer_encoder = CANcoder(steer_encoder_id, canbus)

        if encoder_inverted:
            encoder_config.magnet_sensor.sensor_direction = (
                SensorDirectionValue.CLOCKWISE_POSITIVE
            )
        else:
            encoder_config.magnet_sensor.sensor_direction = (
                SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
            )

        encoder_config.magnet_sensor.magnet_offset = (
            self.offset if RobotBase.isReal() else 0
        )
        self.steer_encoder.configurator.apply(encoder_config)

        steer_config = TalonFXConfiguration()
        steer_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        if steer_inverted:
            steer_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            steer_config.motor_output.inverted = (
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )

        steer_config.current_limits.stator_current_limit = 60
        steer_config.current_limits.stator_current_limit_enable = True

        steer_config.closed_loop_general.continuous_wrap = True
        steer_config.slot0 = self.config.steer_pid_constants

        steer_config.feedback.feedback_sensor_source = (
            FeedbackSensorSourceValue.FUSED_CANCODER
        )
        steer_config.feedback.rotor_to_sensor_ratio = 1 / self.config.steer_ratio
        steer_config.feedback.feedback_remote_sensor_id = steer_encoder_id

        self.steer_motor = TalonFX(steer_motor_id, canbus)
        self.steer_motor.clear_sticky_faults()
        self.steer_motor.configurator.apply(steer_config)

        # self.steer_motor.set_position(self.steer_encoder.get_absolute_position().value)

        self.drive_request = VelocityVoltage(0.0).with_update_freq_hz(250)
        self.steer_request = PositionVoltage(0.0).with_update_freq_hz(250)

        self.io.angle_supplier = self.steer_motor.get_position()
        self.io.absolute_angle_supplier = self.steer_encoder.get_absolute_position()
        self.io.angular_velocity_supplier = self.steer_motor.get_velocity()
        self.io.position_supplier = self.drive_motor.get_position()
        self.io.velocity_supplier = self.drive_motor.get_velocity()
        self.io.steer_supply_voltage_supplier = self.steer_motor.get_motor_voltage()
        self.io.drive_supply_voltage_supplier = self.drive_motor.get_motor_voltage()

        BaseStatusSignal.set_update_frequency_for_all(
            250,
            self.io.position_supplier,
            self.io.velocity_supplier,
            self.io.angle_supplier,
        )

        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.io.angular_velocity_supplier,
            self.io.steer_supply_voltage_supplier,
            self.io.drive_supply_voltage_supplier,
            self.io.absolute_angle_supplier,
        )

        self.io.add_function(self.get_angle, math.degrees)

    def get_drive_position(self) -> units.meters:
        return self.io.position_supplier.value

    def get_drive_velocity(self) -> float:
        return self.io.velocity_supplier.value

    def get_magnitude(self) -> units.meters_per_second:
        return math.hypot(
            self.io.velocity_supplier.value, self.io.angular_velocity_supplier.value
        )

    def get_angle(self) -> units.radians:
        return self.io.absolute_angle_supplier.value * math.tau
        # return (
        #     self.io.angle_supplier.value * math.tau
        #     - math.radians(self.offset) * RobotBase.isReal()
        # )
        # return self.io.angle_supplier.value * math.tau

    def get_module_voltage(self) -> tuple[units.volts, units.volts]:
        return (
            self.io.drive_supply_voltage_supplier.value,
            self.io.steer_supply_voltage_supplier.value,
        )

    def stop(self) -> None:
        self.drive_motor.stopMotor()
        self.steer_motor.stopMotor()

    def reset(self) -> None:
        self.drive_motor.set_position(0)
        self.steer_motor.set_position(0)

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        rot = Rotation2d(self.get_angle())

        desired_state.optimize(rot)
        desired_state.cosineScale(rot)

        ff_drive_volts = self.drive_feed_forward.calculate(desired_state.speed)

        self.drive_motor.set_control(
            self.drive_request.with_velocity(desired_state.speed).with_feed_forward(
                ff_drive_volts
            )
        )
        self.steer_motor.set_control(
            self.steer_request.with_position(desired_state.angle.radians() / math.tau)
        )

    def manually_control(self) -> None:
        return super().manually_control()

    def apply_current_limit(self, current_limit: CurrentLimitsConfigs):
        self.drive_motor.configurator.apply(current_limit)
