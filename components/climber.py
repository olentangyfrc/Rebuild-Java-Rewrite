import math
from enum import Enum
import wpimath.units as units
from magicbot import tunable
import phoenix6
from wpilib import (
    MechanismRoot2d,
)
import wpimath.controller as controller
from phoenix6.controls import VoltageOut
from phoenix6 import configs, signals, hardware
from phoenix6.hardware import TalonFX, CANcoder
from utilities.configs import ClimberConfig
from utilities.IO import ClimberIO
from utilities.helpers import clamp
from utilities.states import ClimberStates
from wpimath.trajectory import TrapezoidProfile, constraint


class Climber:
    # cP = tunable(110)
    # cI = tunable(0.0)
    # cD = tunable(0.0)

    # fS = tunable(0.25)
    # fG = tunable(0.15)
    # fV = tunable(0.0)

    # manual_setpoint = tunable(0.52)

    # manual_voltage = tunable(0.0)

    # UNCOMMENT AFTER PID TUNING, REPLACE WITH NEW VALUES
    MIN_HEIGHT = 0.52
    MAX_HEIGHT = 0.74

    IDLE_HEIGHT = 0.52
    RELEASE_HEIGHT = 0.725
    HOLD_HEIGHT = 0.54

    CLIMB_VOLTAGE = -2

    def __init__(self, config: ClimberConfig):
        self.config = config
        self.climber_motor = TalonFX(self.config.climber_motor_id, config.CANBus)
        self.climber_encoder = CANcoder(
            self.config.climber_encoder_id, config.CANBus
        )  # TODO find can value
        self.io = ClimberIO()
        self._state = ClimberStates.IDLE

        self.motor_config = configs.TalonFXConfiguration()
        self.motor_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.motor_config.motor_output.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.motor_config.current_limits.supply_current_limit = 60
        self.climber_motor.configurator.apply(self.motor_config)

        self.WINCH_REVS_PER_SHAFT_REV = (
            1.0 / 25
        )  # TODO find all constants in this paragraph ahh thing
        self.METERS_PER_WINCH_REV = 1.79
        self.RETRACTED_METERS = 0.52
        self.MAX_HEIGHT_FROM_GROUND = 1.2
        self.TOLERANCE_METERS = 0.02

        self.constraints = TrapezoidProfile.Constraints(9999, 9999)
        self.climbing_PID_controller = controller.ProfiledPIDController(
            self.config.climber_pid_constants.p,
            self.config.climber_pid_constants.i,
            self.config.climber_pid_constants.d,
            self.constraints,
        )
        self.climb_feedforward = controller.ElevatorFeedforward(
            self.config.climber_ff_constants.kS,
            self.config.climber_ff_constants.kG,
            self.config.climber_ff_constants.kV,
        )

        self.climbing_PID_controller.setTolerance(self.TOLERANCE_METERS)

        self.io.target_setpoint = 0.52
        self.climber_voltage_request = VoltageOut(0.0)
        self.climber_encoder.set_position(0.0)

        self._set_up_logging()

    @property
    def state(self) -> ClimberStates:
        return self._state

    def get_height_from_ground(self) -> float:
        return self.get_height_from_retracted() + self.RETRACTED_METERS

    @state.setter
    def state(self, new_state):
        self._state = new_state

    def get_height_from_retracted(self) -> float:
        return (
            self.climber_encoder.get_position().value
            * self.WINCH_REVS_PER_SHAFT_REV
            * self.METERS_PER_WINCH_REV
        )  # TODO REVIEW THIS # 0.019 meters

    def go_to_idle(self) -> None:
        self.state = ClimberStates.IDLE

    def climb(self) -> None:
        self.state = ClimberStates.CLIMB

    def release(self) -> None:
        self.state = ClimberStates.RELEASE

    def hold(self) -> None:
        self.state = ClimberStates.HOLD

    def zero_encoder(self) -> None:
        self.climber_encoder.set_position(0.0)

    def execute(self):
        # UNCOMMENT AFTER TESTING FOR PID VALUES
        self.io.state = self.state.name
        match self.state:
            case ClimberStates.IDLE:
                self.io.target_setpoint = self.IDLE_HEIGHT
            case ClimberStates.RELEASE:
                self.io.target_setpoint = self.RELEASE_HEIGHT
            case ClimberStates.CLIMB:
                self.io.target_setpoint = self.HOLD_HEIGHT
            case ClimberStates.HOLD:
                self.io.target_setpoint = self.HOLD_HEIGHT

        self.io.target_setpoint = clamp(
            self.io.target_setpoint, self.MIN_HEIGHT, self.MAX_HEIGHT
        )

        self.climbing_PID_controller.setGoal(self.io.target_setpoint)

        self.pid_calc = self.climbing_PID_controller.calculate(
            self.get_height_from_ground()
        )
        self.ff_calc = self.climb_feedforward.calculate(
            (self.climbing_PID_controller.getSetpoint().velocity)
        )

        self.io.voltage_output = self.pid_calc + self.ff_calc

        if self.state == ClimberStates.CLIMB:
            self.climber_motor.set_control(
                self.climber_voltage_request.with_output(self.CLIMB_VOLTAGE)
            )
            if self.climbing_PID_controller.atSetpoint():
                self.hold()
        elif self.state == ClimberStates.HOLD:
            self.climber_motor.set_control(self.climber_voltage_request.with_output(0))
        else:
            self.climber_motor.set_control(
                self.climber_voltage_request.with_output(self.io.voltage_output)
            )

    def _set_up_logging(self):
        self.io.add_function(self.get_height_from_ground)
