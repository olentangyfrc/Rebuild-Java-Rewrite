import math
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.controls import (
    VoltageOut,
    VelocityVoltage,
    MotionMagicVelocityVoltage,
)  # Added VelocityVoltage
from phoenix6.hardware import ParentDevice, TalonFX, CANcoder
from phoenix6.configs import CurrentLimitsConfigs, TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls.follower import Follower, MotorAlignmentValue

import wpilib
from wpilib import SmartDashboard, RobotBase, MechanismRoot2d, Timer
from wpimath.controller import ArmFeedforward, ProfiledPIDController
from wpimath.filter import SlewRateLimiter, MedianFilter
from wpimath.trajectory import TrapezoidProfile
from wpimath import units

from utilities.configs import IntakeConfig
from utilities.helpers import clamp, SimplePControllerSim, calculate_power
from utilities.IO import IntakeIO
from utilities.states import IntakeStates

from magicbot import tunable

MAX_PIVOT_POSITION = math.radians(116)
MIN_PIVOT_POSITION = math.radians(0.5)

# we'll probably mess with these a lot, so I made them constants
AGITATE_UPPER_ANGLE = MAX_PIVOT_POSITION - math.radians(16)

AGITATION_AMPLITUDE = math.radians(40)
FULL_HOPPER_DELAY = 0.2
AGITATION_FREQUENCY = 0.75


class Intake:
    manual = False
    # kg = tunable(0.42)
    stall_current = 70

    def __init__(self, config: IntakeConfig, mech_root: MechanismRoot2d):
        self.roller_motor_configs = TalonFXConfiguration()
        self.roller_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.roller_motor_configs.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
        )

        # Configure PID for Velocity Control (Slot 0)
        self.slot0_configs = self.roller_motor_configs.slot0
        self.slot0_configs.k_s = config.intake_roller_ff_constants.kS
        self.slot0_configs.k_v = config.intake_roller_ff_constants.kV
        self.slot0_configs.k_a = config.intake_roller_ff_constants.kA
        self.slot0_configs.k_p = config.intake_roller_pid_constants.p
        self.slot0_configs.k_i = config.intake_roller_pid_constants.i
        self.slot0_configs.k_d = config.intake_roller_pid_constants.d

        self.motion_magic_configs = self.roller_motor_configs.motion_magic
        self.motion_magic_configs.motion_magic_acceleration = 9999

        self.io = IntakeIO()
        self.config = config

        self.left_roller_motor = TalonFX(config.intake_roller_id[1], config.CANbus)
        self.left_roller_motor.configurator.apply(self.roller_motor_configs)

        self.right_roller_motor = TalonFX(config.intake_roller_id[0], config.CANbus)
        self.right_roller_motor.configurator.apply(self.roller_motor_configs)

        self.right_roller_motor.set_control(
            Follower(config.intake_roller_id[1], MotorAlignmentValue.OPPOSED)
        )

        self._state = IntakeStates.DEPLOYED

        pivot_motor_config = TalonFXConfiguration()
        pivot_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        pivot_motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.pivot_motor = TalonFX(config.pivot_motor_id, config.CANbus)
        self.pivot_motor.configurator.apply(pivot_motor_config)

        self.pivot_encoder = CANcoder(config.pivot_encoder_id, config.CANbus)

        self.pivot_ff = ArmFeedforward(
            config.pivot_ff_constants.kS,
            config.pivot_ff_constants.kG
            if config.pivot_ff_constants.kG is not None
            else 0.0,
            config.pivot_ff_constants.kV,
            config.pivot_ff_constants.kA,
        )

        constraints = TrapezoidProfile.Constraints(
            config.pivot_profile_constants.max_vel,
            config.pivot_profile_constants.max_acc,
        )

        self.pivot_pid = ProfiledPIDController(
            config.pivot_pid_constants.p,
            config.pivot_pid_constants.i,
            config.pivot_pid_constants.d,
            constraints,
        )

        self.pivot_pid.setTolerance(math.radians(5))

        self.io.pivot_position_supplier = self.pivot_encoder.get_absolute_position()
        self.io.pivot_velocity_supplier = self.pivot_encoder.get_velocity()

        self.io.pivot_stator_current_supplier = self.pivot_motor.get_stator_current()
        self.io.pivot_supply_current_supplier = self.pivot_motor.get_supply_current()
        self.io.pivot_supply_voltage_supplier = self.pivot_motor.get_supply_voltage()

        self.io.right_intake_roller_stator_current_supplier = (
            self.left_roller_motor.get_stator_current()
        )
        self.io.right_intake_roller_supply_current_supplier = (
            self.left_roller_motor.get_supply_current()
        )

        self.io.right_intake_roller_velocity_supplier = (
            self.left_roller_motor.get_velocity()
        )

        self.io.pivot_torque_current_supplier = self.pivot_motor.get_torque_current()

        self.pivot_torque_current_filter = MedianFilter(15)

        self.io.left_intake_roller_stator_current_supplier = (
            self.right_roller_motor.get_stator_current()
        )
        self.io.left_intake_roller_supply_current_supplier = (
            self.right_roller_motor.get_supply_current()
        )

        self.io.left_intake_roller_velocity_supplier = (
            self.right_roller_motor.get_velocity()
        )

        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.io.pivot_position_supplier,
            self.io.pivot_velocity_supplier,
            self.io.right_intake_roller_velocity_supplier,
            self.io.pivot_stator_current_supplier,
            self.io.pivot_supply_current_supplier,
            self.io.pivot_supply_voltage_supplier,
            self.io.right_intake_roller_stator_current_supplier,
            self.io.right_intake_roller_supply_current_supplier,
            self.io.pivot_torque_current_supplier,
            self.io.left_intake_roller_stator_current_supplier,
            self.io.left_intake_roller_supply_current_supplier,
        )

        # Roller request changed to Velocity
        self.manual_override_voltage = 0.0
        self.manual_override_flag = False

        self.intake_velocity_request = MotionMagicVelocityVoltage(0.0)
        self.pivot_voltage_request = VoltageOut(0.0)
        self.roller_stop_request = VoltageOut(0.0)

        self._set_up_logging()

        if RobotBase.isSimulation():
            self.pivot_sim_pid = SimplePControllerSim(
                self.pivot_pid,
                limiter=SlewRateLimiter(2),
                initial_value=self.io.target_pivot_angle,
            )

        self.pivot_pid.reset(self.get_pivot_angle())

        self.mech_ligament = mech_root.appendLigament(
            "Intake", 1, math.degrees(self.get_pivot_angle())
        )

        self._full_hopper_flag = True

        # ParentDevice.optimize_bus_utilization_for_all(
        #     self.pivot_motor, self.intake_roller_motor, self.pivot_encoder
        # )

        self.agitation_timer = Timer()

        self._high_eject = False

    @property
    def state(self) -> IntakeStates:
        return self._state

    @state.setter
    def state(self, new_state: IntakeStates) -> None:
        if not (isinstance(new_state, IntakeStates) or new_state is None):
            raise ValueError("State must be a IntakeState")
        self._state = new_state

    def get_pivot_angle(self) -> units.radians:
        if RobotBase.isSimulation():
            return self.pivot_sim_pid.value

        angle = (
            self.io.pivot_position_supplier.value
            * self.config.pivot_gear_ratio
            * math.tau
        ) - math.radians(self.config.encoder_offset)

        if angle < math.radians(-15):
            angle += math.radians(159.8)

        return angle

    def get_pivot_velocity(self) -> units.radians_per_second:
        return (
            self.io.pivot_velocity_supplier.value
            * self.config.pivot_gear_ratio
            * math.tau
        )

    def get_roller_velocity(self) -> float:
        return (
            self.left_roller_motor.get_velocity().value
            + self.right_roller_motor.get_velocity().value
        ) / 2

    def at_target_position(self) -> bool:
        return abs(self.get_pivot_angle() - self.io.target_pivot_angle) <= math.radians(
            5
        )

    def is_stalled(self) -> bool:
        return (
            abs(self.get_pivot_velocity()) < 0.1
            and self.io.pivot_stator_current_supplier.value > self.stall_current
        )

    def get_pivot_torque_amps(self) -> units.amperes:
        return self.pivot_torque_current_filter.calculate(
            self.io.pivot_torque_current_supplier.value
        )

    def agitate(self) -> None:
        if self.state not in [
            IntakeStates.AGITATE,
        ]:
            self.agitation_timer.stop()
            self.agitation_timer.reset()
            self._full_hopper_flag = False
            self.state = IntakeStates.AGITATE

    def intake(self) -> None:
        self.state = IntakeStates.INTAKE

    def intake_pass(self) -> None:
        self.state = IntakeStates.INTAKE_PASS

    def deploy(self) -> None:
        self.state = IntakeStates.DEPLOYED

    def go_to_idle(self) -> None:
        self.state = IntakeStates.IDLE

    def eject(self) -> None:
        self.state = IntakeStates.EJECT

    def stash(self) -> None:
        self.state = IntakeStates.STASHED

    def emergency_stash(self) -> None:
        self.state = IntakeStates.EMERGENCY_STASH

    def manual_override(self, manual_override_volts: units.volts) -> None:
        self.manual_override_flag = True
        self.manual_override_voltage = manual_override_volts

    def apply_current_limtis(self, current_limits: CurrentLimitsConfigs) -> None:
        self.left_roller_motor.configurator.apply(current_limits)
        self.right_roller_motor.configurator.apply(current_limits)

    def execute(self):
        if self.manual and not self.io.tuning_sendables_sent:
            self._send_tuning_sendables()
            self.io.tuning_sendables_sent = True

        # if self.io.tuning_sendables_sent:
        #     self.pivot_ff.setKg(self.kg)

        self._handle_state_logic()

        if self.manual:
            self.io.target_pivot_angle = self.pivot_pid.getGoal().position

        self.pivot_pid.setGoal(
            clamp(self.io.target_pivot_angle, MIN_PIVOT_POSITION, MAX_PIVOT_POSITION)
        )

        if RobotBase.isSimulation():
            self.pivot_sim_pid.update()
        else:
            # Pivot Logic
            if self.manual_override_flag:
                self.io.pivot_voltage = self.manual_override_voltage
            else:
                self.io.pivot_voltage = self.pivot_pid.calculate(
                    self.get_pivot_angle()
                ) + self.pivot_ff.calculate(self.get_pivot_angle(), 0)

            self.pivot_motor.set_control(
                self.pivot_voltage_request.with_output(
                    self.io.pivot_voltage
                ).with_enable_foc(True)
            )

        # Roller Velocity Control
        # Values are now interpreted as Rotations Per Second
        if self.io.target_intake_roller_velocity == 0 or (
            self.get_pivot_angle() > math.radians(42) and not self.manual_override_flag
        ):
            self.left_roller_motor.set_control(
                self.roller_stop_request.with_enable_foc(True).with_output(0)
            )
        else:
            self.left_roller_motor.set_control(
                self.intake_velocity_request.with_velocity(
                    self.io.target_intake_roller_velocity
                ).with_enable_foc(True)
            )

        self.mech_ligament.setAngle(math.degrees(self.get_pivot_angle()))

    def _handle_state_logic(self) -> None:
        self.io.state = self.state.name

        match self.state:
            case IntakeStates.IDLE:
                self.io.target_intake_roller_velocity = 0
                self.io.target_pivot_angle = MAX_PIVOT_POSITION - math.radians(1)
                self._high_eject = False
            case IntakeStates.DEPLOYED:
                self.io.target_pivot_angle = math.radians(-1)
                self.io.target_intake_roller_velocity = (
                    -5
                )  # To stop the fuel from leaking out
                self._high_eject = False
            case IntakeStates.INTAKE:
                self.io.target_intake_roller_velocity = -90
                self.io.target_pivot_angle = math.radians(-1)

                self._high_eject = False
            case IntakeStates.INTAKE_PASS:
                if self.get_pivot_angle() < math.radians(30):
                    self.io.target_intake_roller_velocity = (
                        -40
                    )  # Move at about 40% speed
                self.io.target_pivot_angle = 0

                self._high_eject = False

            case IntakeStates.AGITATE:
                # self.io.target_pivot_angle = math.radians(30)

                # if self.get_pivot_torque_amps() > 35:  # check for full hopper
                #     self._full_hopper_flag = True

                # if self.get_pivot_angle() > math.radians(20):
                #     if self._full_hopper_flag:
                #         if not self.agitation_timer.isRunning():
                #             self.agitation_timer.start()

                #         if self.agitation_timer.get() > 0.9:
                #             self.io.target_pivot_angle = 60

                #             if self.get_pivot_angle() > math.radians(55):
                #                 self.stash()
                #                 self.agitation_timer.stop()
                #                 self.agitation_timer.reset()
                #     else:
                #         self.stash()

                if not self.agitation_timer.isRunning():
                    self.agitation_timer.restart()

                if self.agitation_timer.get() > 1.5:
                    self.stash()

                    self.agitation_timer.stop()
                    self.agitation_timer.reset()
                elif self.agitation_timer.get() > 1:
                    self.io.target_pivot_angle = math.radians(60)
                elif self.agitation_timer.get() > 0.65:
                    self.io.target_pivot_angle = math.radians(25)

                self.io.target_intake_roller_velocity = -15

            case IntakeStates.EJECT:
                self.io.target_intake_roller_velocity = 90
                self.io.target_pivot_angle = 0
            case IntakeStates.STASHED:
                self.io.target_intake_roller_velocity = 0
                self.io.target_pivot_angle = MAX_PIVOT_POSITION - math.radians(1)
            case IntakeStates.EMERGENCY_STASH:
                self.io.target_intake_roller_velocity = -30
                self.io.target_pivot_angle = MAX_PIVOT_POSITION - math.radians(1)

    def _send_tuning_sendables(self) -> None:
        SmartDashboard.putData("Pivot PID", self.pivot_pid)

    def _set_up_logging(self) -> None:
        self.io.add_function(self.get_pivot_angle, math.degrees)
        # self.io.add_function(self.get_pivot_angle, alternate_name="Pivot angle rad")
        self.io.add_function(self.at_target_position)
        # self.io.add_function(self.get_pivot_velocity)
        self.io.add_function(self.is_stalled)
        self.io.add_function(self.get_roller_velocity)

        self.io.add_function(
            lambda: calculate_power(
                self.io.pivot_supply_current_supplier,
                self.io.pivot_supply_voltage_supplier,
            ),
            alternate_name="Pivot Power Draw",
        )

        self.io.add_function(self.get_pivot_torque_amps)
