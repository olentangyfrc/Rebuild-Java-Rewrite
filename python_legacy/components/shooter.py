import math

import wpimath.units as units
from magicbot import tunable
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.configs import (
    CANrangeConfiguration,
    TalonFXConfiguration,
    CurrentLimitsConfigs,
)
from phoenix6.controls import VoltageOut, MotionMagicVelocityVoltage
from phoenix6.controls.follower import Follower, MotorAlignmentValue
from phoenix6.hardware import CANcoder, CANrange, TalonFX, ParentDevice
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpilib import (
    DriverStation,
    SmartDashboard,
    Timer,
    RobotBase,
    MechanismRoot2d,
    Color8Bit,
)
from wpimath import controller
from wpimath.filter import Debouncer, MedianFilter

from components.drivetrain import Drivetrain
from components.drivetrain import Drivetrain
from utilities.configs import ShooterConfig
from utilities.helpers import clamp, SimplePControllerSim, calculate_power
from utilities.states import ShooterStates
from utilities.IO import ShooterIO

# TODO: Figure out whether to use rpm or mps

MAX_SHOOTER_RPM = 4000
MIN_HOOD_ANGLE = math.radians(2)
MAX_HOOD_ANGLE = math.radians(47)


class Shooter:
    manual_indexer1_rps = 0.0
    manual_indexer2_rps = 0.0
    manual_target_rpm = 0
    manual_hood_angle = 2.0

    # manual_shoot_rpm = tunable(0)

    manual = False
    # kg = tunable(0.0)

    drivetrain: Drivetrain

    def __init__(self, config: ShooterConfig, mech_root: MechanismRoot2d):
        self.config = config
        self.io = ShooterIO()

        self.shoot_motor_config = TalonFXConfiguration()
        self.shoot_motor_config.motor_output.neutral_mode = NeutralModeValue.COAST

        self.right_bottom = TalonFX(
            self.config.shooter_motor_ids[1], self.config.CANbus
        )
        self.right_bottom.configurator.apply(self.shoot_motor_config)

        self.left_top = TalonFX(self.config.shooter_motor_ids[2], self.config.CANbus)
        self.left_top.configurator.apply(self.shoot_motor_config)

        self.right_top = TalonFX(self.config.shooter_motor_ids[0], self.config.CANbus)
        self.right_top.configurator.apply(self.shoot_motor_config)

        self.shoot_motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.slot0_shoot_configs = self.shoot_motor_config.slot0
        self.slot0_shoot_configs.k_p = self.config.shoot_pid_constants.p
        self.slot0_shoot_configs.k_i = self.config.shoot_pid_constants.i
        self.slot0_shoot_configs.k_d = self.config.shoot_pid_constants.d
        self.slot0_shoot_configs.k_s = self.config.shoot_ff_constants.kS
        self.slot0_shoot_configs.k_v = self.config.shoot_ff_constants.kV
        self.slot0_shoot_configs.k_a = self.config.shoot_ff_constants.kA

        self.motion_magic_configs = self.shoot_motor_config.motion_magic
        self.motion_magic_configs.motion_magic_acceleration = 9999

        self.left_bottom = TalonFX(self.config.shooter_motor_ids[3], self.config.CANbus)
        self.left_bottom.configurator.apply(self.shoot_motor_config)

        self.indexer_configs = TalonFXConfiguration()
        self.indexer_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.indexer_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.slot0_feed_configs = self.indexer_configs.slot0
        self.slot0_feed_configs.k_s = self.config.indexer1_ff_constants.kS
        self.slot0_feed_configs.k_v = self.config.indexer1_ff_constants.kV
        self.slot0_feed_configs.k_a = self.config.indexer1_ff_constants.kA
        self.slot0_feed_configs.k_p = self.config.indexer1_pid_constants.p
        self.slot0_feed_configs.k_i = self.config.indexer1_pid_constants.i
        self.slot0_feed_configs.k_d = self.config.indexer1_pid_constants.d
        self.motion_magic_configs = self.indexer_configs.motion_magic
        self.motion_magic_configs.motion_magic_acceleration = 9999

        self.indexer1 = TalonFX(self.config.indexer_id[0], self.config.CANbus)
        self.indexer1.configurator.apply(self.indexer_configs)

        self.slot0_feed_configs = self.indexer_configs.slot0
        self.slot0_feed_configs.k_s = self.config.indexer2_ff_constants.kS
        self.slot0_feed_configs.k_v = self.config.indexer2_ff_constants.kV
        self.slot0_feed_configs.k_a = self.config.indexer2_ff_constants.kA
        self.slot0_feed_configs.k_p = self.config.indexer2_pid_constants.p
        self.slot0_feed_configs.k_i = self.config.indexer2_pid_constants.i
        self.slot0_feed_configs.k_d = self.config.indexer2_pid_constants.d
        self.motion_magic_configs = self.indexer_configs.motion_magic
        self.motion_magic_configs.motion_magic_acceleration = 9999

        self.indexer2 = TalonFX(self.config.indexer_id[1], self.config.CANbus)
        self.indexer2.configurator.apply(self.indexer_configs)

        self.hood_motor_configs = TalonFXConfiguration()
        self.hood_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.hood_motor_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.hood_motor = TalonFX(self.config.hood_id, self.config.CANbus)
        self.hood_motor.configurator.apply(self.hood_motor_configs)
        self.hood_encoder = CANcoder(self.config.hood_encoder_id, self.config.CANbus)

        # canrange_config = CANrangeConfiguration()
        # canrange_config.proximity_params.proximity_threshold = 0.36

        # self.left_canrange = CANrange(self.config.left_canrange, self.config.CANbus)
        # self.right_canrange = CANrange(self.config.right_canrange, self.config.CANbus)
        # self.left_canrange.configurator.apply(canrange_config)
        # self.right_canrange.configurator.apply(canrange_config)

        self.median_filer = MedianFilter(15)

        self.flywheel_debouncer = Debouncer(0.1, Debouncer.DebounceType.kRising)
        self.canrange_debouncer = Debouncer(0.75, Debouncer.DebounceType.kBoth)

        self._state = ShooterStates.IDLE

        self.left_top.set_control(
            Follower(self.left_bottom.device_id, MotorAlignmentValue.ALIGNED)
        )
        self.right_bottom.set_control(
            Follower(self.left_bottom.device_id, MotorAlignmentValue.OPPOSED)
        )
        self.right_top.set_control(
            Follower(self.left_bottom.device_id, MotorAlignmentValue.OPPOSED)
        )  # need to recheck motoralignmentvalues
        self.hood_ff = controller.ArmFeedforward(
            self.config.hood_ff_constants.kS,
            self.config.hood_ff_constants.kG,  # type: ignore
            self.config.hood_ff_constants.kV,
        )

        self.hood_pid = controller.PIDController(
            self.config.hood_pid_constants.p,
            self.config.hood_pid_constants.i,
            self.config.hood_pid_constants.d,
        )
        self.hood_pid.setTolerance(math.radians(0.5))
        self.hood_pid.setIZone(math.radians(3))
        self.hood_pid.setSetpoint(MIN_HOOD_ANGLE + math.radians(2))
        self.hood_pid.reset()

        self.io.left_bottom_velocity_supplier = self.left_bottom.get_velocity()
        self.io.left_top_velocity_supplier = self.left_top.get_velocity()
        self.io.right_bottom_velocity_supplier = self.right_bottom.get_velocity()
        self.io.right_top_velocity_supplier = self.right_top.get_velocity()

        self.io.left_bottom_supply_current_supplier = (
            self.left_bottom.get_supply_current()
        )
        self.io.left_top_supply_current_supplier = self.left_top.get_supply_current()
        self.io.right_bottom_supply_current_supplier = (
            self.right_bottom.get_supply_current()
        )
        self.io.right_top_supply_current_supplier = self.right_top.get_supply_current()

        self.io.left_bottom_stator_current_supplier = (
            self.left_bottom.get_stator_current()
        )
        self.io.left_top_stator_current_supplier = self.left_top.get_stator_current()
        self.io.right_bottom_stator_current_supplier = (
            self.right_bottom.get_stator_current()
        )
        self.io.right_top_stator_current_supplier = self.right_top.get_stator_current()

        self.io.left_bottom_supply_voltage_supplier = (
            self.left_bottom.get_supply_voltage()
        )
        self.io.left_top_supply_voltage_supplier = self.left_top.get_supply_voltage()
        self.io.right_bottom_supply_voltage_supplier = (
            self.right_bottom.get_supply_voltage()
        )
        self.io.right_top_supply_voltage_supplier = self.right_top.get_supply_voltage()

        self.io.hood_position_supplier = self.hood_encoder.get_absolute_position()
        self.io.hood_supply_current_supplier = self.hood_motor.get_supply_current()
        self.io.hood_supply_voltage_supplier = self.hood_motor.get_supply_voltage()

        self.io.feeder_supply_current_supplier = self.indexer1.get_supply_current()
        self.io.feeder_torque_current_supplier = self.indexer1.get_torque_current()
        self.io.feeder_supply_voltage_supplier = self.hood_motor.get_supply_voltage()
        self.io.feeder_stator_current_supplier = self.indexer1.get_stator_current()

        self.io.indexer1_rps_supplier = self.indexer1.get_velocity()
        self.io.indexer2_rps_supplier = self.indexer2.get_velocity()

        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.io.left_bottom_supply_current_supplier,
            self.io.left_top_velocity_supplier,
            self.io.right_bottom_supply_current_supplier,
            self.io.right_top_supply_current_supplier,
            self.io.left_bottom_stator_current_supplier,
            self.io.left_top_stator_current_supplier,
            self.io.right_bottom_stator_current_supplier,
            self.io.right_top_stator_current_supplier,
            self.io.left_bottom_supply_voltage_supplier,
            self.io.left_top_supply_voltage_supplier,
            self.io.right_bottom_supply_voltage_supplier,
            self.io.right_top_supply_voltage_supplier,
            self.io.hood_supply_current_supplier,
            self.io.feeder_supply_current_supplier,
            self.io.feeder_supply_voltage_supplier,
            self.io.feeder_supply_voltage_supplier,
            self.io.feeder_supply_current_supplier,
            self.io.feeder_stator_current_supplier,
            self.io.indexer1_rps_supplier,
            self.io.indexer2_rps_supplier,
        )

        BaseStatusSignal.set_update_frequency_for_all(
            100, self.io.feeder_torque_current_supplier
        )

        # 100 works fine
        BaseStatusSignal.set_update_frequency_for_all(
            250,
            self.io.left_bottom_velocity_supplier,
            self.io.left_top_velocity_supplier,
            self.io.right_bottom_velocity_supplier,
            self.io.right_top_velocity_supplier,
            self.io.hood_position_supplier,
        )

        self.shooter_voltage_request = VoltageOut(0.0)
        self.shooter_motion_magic_request = MotionMagicVelocityVoltage(0.0)
        self.shooter_motion_magic_request.update_freq_hz = 250

        self.feeder_motion_magic_request = MotionMagicVelocityVoltage(0.0)
        self.hood_voltage_request = VoltageOut(0.0)

        self._hood_is_low = True

        self._shoot_rpm = 1500
        self._shoot_angle = math.radians(20)
        self._warm_up_rpm = 1000

        self._is_low_ceiling = False
        self._passing = False
        self._hood_locked = False

        self._hood_offset = self.config.hood_offset
        self.io.clamped_rpm = self.io.target_rpm

        self._set_up_logging()
        self.shot_timer = Timer()
        self.initial_feed_timer = Timer()
        self.jam_detection_timer = Timer()
        self.dry_fire_timer = Timer()

        if RobotBase.isSimulation():
            self.hood_pid_sim = SimplePControllerSim(
                self.hood_pid, None, self.io.target_hood_angle
            )

        self.ligament = mech_root.appendLigament(
            "Hood", 1, math.degrees(self.get_hood_angle()), color=Color8Bit(0, 0, 255)
        )

    @property
    def state(self) -> ShooterStates | None:
        return self._state

    @state.setter
    def state(self, new_state: ShooterStates | None) -> None:
        if not (isinstance(new_state, ShooterStates) or new_state is None):
            raise ValueError("State must be a ShooterState")
        self._state = new_state

    def lock_hood(self) -> None:
        self._hood_locked = True

    def unlock_hood(self) -> None:
        self._hood_locked = False

    def get_average_rps(self) -> units.turns_per_second:
        return (
            self.io.left_bottom_velocity_supplier.value
            + self.io.left_top_velocity_supplier.value
            + self.io.right_bottom_velocity_supplier.value
            + self.io.right_top_velocity_supplier.value
        ) / 4

    def get_flywheel_speed_rpm(self) -> units.revolutions_per_minute:
        return self.get_average_rps() * self.config.shooter_gear_ratio * 60
        # check for shooter gear ratio

    def get_flywheel_speed_filtered(self) -> units.revolutions_per_minute:
        return self.median_filer.calculate(self.get_flywheel_speed_rpm())

    def get_indexer1_velocity(self) -> float:
        return self.io.indexer1_rps_supplier.value

    def get_indexer2_velocity(self) -> float:
        return self.io.indexer2_rps_supplier.value

    def get_stator_current(self) -> units.amperes:
        stator = (
            self.left_bottom.get_stator_current().value_as_double
            + self.left_top.get_stator_current().value_as_double
            + self.right_bottom.get_stator_current().value_as_double
            + self.right_top.get_stator_current().value_as_double
        )
        return stator / 4

    def get_supply_current(self) -> units.amperes:
        supply = (
            self.left_bottom.get_supply_current().value_as_double
            + self.left_top.get_supply_current().value_as_double
            + self.right_bottom.get_supply_current().value_as_double
            + self.right_top.get_supply_current().value_as_double
        )
        return supply / 4

    def get_supply_voltage(self) -> units.volts:
        supply_volts = (
            self.io.left_bottom_supply_voltage_supplier.value
            + self.io.left_top_supply_voltage_supplier.value
            + self.io.right_bottom_supply_voltage_supplier.value
            + self.io.right_top_stator_current_supplier.value
        )
        return supply_volts / 4

    def get_hood_angle(self) -> units.radians:
        # Don't worry about all this. This too complicated to explain so just ask Akshaj.
        # Akshaj is finding a better way to do this, this is temporary.

        # Akshaj we are gonna have to redo this
        if RobotBase.isSimulation():
            return self.hood_pid_sim.value

        angle = (
            self.io.hood_position_supplier.value
        ) * self.config.hood_gear_ratio * math.tau - math.radians(self._hood_offset)

        # if self._hood_is_low and angle < math.radians(19):
        #     self._hood_is_low = False
        # elif (not self._hood_is_low) and angle > math.radians(28):
        #     self._hood_is_low = True

        # if not self._hood_is_low:
        #     angle += math.radians(26.5)
        return angle

    def get_absolute_hood_position(self) -> units.radians:
        # Don't worry about all this. This too complicated to explain so just ask Akshaj.
        # Akshaj is finding a better way to do this, this is temporary.

        # Akshaj we are gonna have to redo this
        angle = (
            (self.io.hood_position_supplier.value)
            * self.config.hood_gear_ratio
            * math.tau
        )

        return angle

    def get_hood_stall_current(self) -> units.amperes:
        return self.hood_motor.get_stator_current().value

    def get_indexer1_torque_current(self) -> units.amperes:
        return self.io.feeder_torque_current_supplier.value

    def is_dry_firing(self) -> bool:
        return self.state in [ShooterStates.SHOOT] and self.dry_fire_timer.get() > 0.75

    def at_target_speed(self) -> bool:
        if self.io.clamped_rpm == 0:
            return False

        # return self.flywheel_debouncer.calculate(
        #     abs(self.get_flywheel_speed_rpm() - self.io.clamped_rpm)
        #     / self.io.clamped_rpm
        #     < 0.015
        # )

        return self.flywheel_debouncer.calculate(
            abs(self.left_bottom.get_closed_loop_error().value_as_double)
            * 60
            / self.io.clamped_rpm
            < 0.015
        )

    def reset_hood(self) -> None:
        self._hood_is_low = False

    def set_shoot_rpm(self, rpm: units.revolutions_per_minute) -> None:
        self._shoot_rpm = rpm

    def set_shoot_angle(self, angle: units.radians) -> None:
        self._shoot_angle = angle

    # The bottom two methods are helpers for interpolation
    def change_shoot_rpm(self, delta_rpm: units.revolutions_per_minute) -> None:
        if self.manual:
            self.manual_target_rpm += delta_rpm

        self._shoot_rpm += delta_rpm

    def change_shoot_angle(self, delta_angle: units.radians) -> None:
        if self.manual:
            self.manual_hood_angle += delta_angle

        self._shoot_angle += delta_angle

    def shoot(self, passing: bool = False) -> None:
        self._passing = passing
        self.state = ShooterStates.SPIN_UP

    def warm_up_shooter(self, warm_up_rpm: units.revolutions_per_minute = 1200) -> None:
        if self.state not in [ShooterStates.SPIN_UP, ShooterStates.SHOOT]:
            self._warm_up_rpm = warm_up_rpm
            self.state = ShooterStates.WARM_UP

    def cooldown(self) -> None:
        self.state = ShooterStates.COOLDOWN

    def go_to_idle(self) -> None:
        self.state = ShooterStates.IDLE

    def eject(self) -> None:
        self.state = ShooterStates.EJECT

    def apply_current_limits(
        self,
        shoot_current_limit: CurrentLimitsConfigs,
        indexer1_current_limit: CurrentLimitsConfigs,
        indexer2_current_limit: CurrentLimitsConfigs,
    ) -> None:
        self.left_bottom.configurator.apply(shoot_current_limit)
        self.left_top.configurator.apply(shoot_current_limit)
        self.right_bottom.configurator.apply(shoot_current_limit)
        self.right_top.configurator.apply(shoot_current_limit)

        # current limit will need to be lowered

        self.indexer1.configurator.apply(indexer1_current_limit)
        self.indexer2.configurator.apply(indexer2_current_limit)

    def execute(self):
        self._handle_state_logic()

        if RobotBase.isSimulation():
            self.hood_pid_sim.update()

        if self.manual:
            self.state = None

            self.io.target_rpm = self.manual_target_rpm
            self.io.target_indexer1_rps = self.manual_indexer1_rps
            self.io.target_indexer2_rps = self.manual_indexer2_rps
            # self.io.target_hood_angle = math.radians(self.manual_hood_angle)
            # self.hood_ff.setKg(
            #     self.kg
            # )
            # self.hood_pid.setSetpoint(math.radians(self.manual_hood_angle))

            if not self.io.tuning_sendables_sent:
                self._send_tuning_sendables()
                self.io.tuning_sendables_sent = True

        self.io.state = self.state.name if self.state is not None else "None"

        # DO NOT TOUCH THESE LINES OF CODE UNLESS WE WANT A HOLE IN OUR CEILING
        theta = math.degrees(self.io.target_hood_angle)
        max_rpm = 0.578583 * (theta**2) - (12.35148 * theta) + 1300
        max_rpm = max_rpm if max_rpm < MAX_SHOOTER_RPM else MAX_SHOOTER_RPM

        if self._is_low_ceiling:
            self.io.clamped_rpm = clamp(self.io.target_rpm, -max_rpm, max_rpm)
        else:
            self.io.clamped_rpm = self.io.target_rpm

        if self._hood_locked:
            self.io.target_hood_angle = MIN_HOOD_ANGLE
        else:
            self.io.target_hood_angle = clamp(
                self.io.target_hood_angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE
            )

        self.hood_pid.setSetpoint(self.io.target_hood_angle)
        self.io.hood_voltage = self.hood_pid.calculate(self.get_hood_angle())
        self.io.hood_voltage += self.hood_ff.calculate(self.get_hood_angle(), 0)

        if self.io.clamped_rpm != 0:
            shoot_velocity = self.io.clamped_rpm / self.config.shooter_gear_ratio
            shoot_velocity /= 60
            self.left_bottom.set_control(
                self.shooter_motion_magic_request.with_velocity(
                    shoot_velocity
                ).with_enable_foc(True)
            )
        else:
            self.left_bottom.set_control(
                self.shooter_voltage_request.with_output(
                    0  # Stops any sudden braking from pid loop
                ).with_enable_foc(True)
            )

        self.indexer1.set_control(
            self.feeder_motion_magic_request.with_velocity(
                self.io.target_indexer1_rps
            ).with_enable_foc(False)
        )
        self.indexer2.set_control(
            self.feeder_motion_magic_request.with_velocity(
                self.io.target_indexer2_rps
            ).with_enable_foc(True)
        )

        if (
            self.get_hood_angle() < MAX_HOOD_ANGLE + math.radians(2)
            # and not self.hood_pid.atSetpoint()
        ):
            if self.hood_pid.atSetpoint():
                self.hood_pid.reset()
            self.hood_motor.set_control(
                self.hood_voltage_request.with_output(
                    self.io.hood_voltage
                ).with_enable_foc(False)
            )
        else:
            self.hood_motor.set_control(
                self.hood_voltage_request.with_output(-1).with_enable_foc(False)
            )

        self.ligament.setAngle(math.degrees(self.get_hood_angle()))

    def _handle_state_logic(self) -> None:
        match self.state:
            case ShooterStates.IDLE:
                self.io.target_rpm = 0
                self.io.target_indexer1_rps = 0
                self.io.target_indexer2_rps = 0
                self.io.target_hood_angle = math.radians(2)
            case ShooterStates.WARM_UP:
                self.io.target_rpm = self._warm_up_rpm
                self.io.target_hood_angle = math.radians(2)
            case ShooterStates.SPIN_UP:
                self.io.target_indexer1_rps = -30
                self.io.target_indexer2_rps = -5
                self.io.target_rpm = self._shoot_rpm
                # self.io.target_rpm = self.manual_shoot_rpm
                self.io.target_hood_angle = self._shoot_angle
                # self.io.target_hood_angle = math.radians(self.manual_hood_angle)

                if not self.initial_feed_timer.isRunning():
                    self.initial_feed_timer.restart()

                if (
                    self.at_target_speed()  # and self.initial_feed_timer.get() > 0.2
                ):  # required feed time of 0.2 seconds which is the unjam time
                    self.state = ShooterStates.SHOOT
                    self.dry_fire_timer.restart()
                    self.jam_detection_timer.stop()
                    self.jam_detection_timer.reset()
            case ShooterStates.SHOOT:
                if (
                    self.get_indexer1_torque_current() > 100
                    and self.get_indexer1_velocity() < 70
                ):  # gave it a second to confirm jamming
                    if not self.jam_detection_timer.isRunning():
                        self.jam_detection_timer.restart()
                    elif self.jam_detection_timer.get() > 0.35:
                        print("oh my god!!! I'm jamming!!! please save me Ani!\n" * 10)
                        self.state = ShooterStates.SPIN_UP
                else:
                    if self.jam_detection_timer.isRunning():
                        self.jam_detection_timer.stop()
                        self.jam_detection_timer.reset()

                    if self.initial_feed_timer.isRunning():
                        self.initial_feed_timer.stop()
                        self.initial_feed_timer.reset()

                    self.io.target_indexer1_rps = 60 if not self._passing else 30
                    self.io.target_indexer2_rps = 70 if not self._passing else 35

                    self.io.target_rpm = self._shoot_rpm
                    # self.io.target_rpm = self.manual_shoot_rpm
                    self.io.target_hood_angle = self._shoot_angle
                    # self.io.target_hood_angle = math.radians(self.manual_hood_angle)

                    if not self.shot_timer.isRunning():
                        self.shot_timer.start()

                    # if self.io.right_detection_supplier.value:
                    #     self.dry_fire_timer.restart()

                # TODO: Uncomment these bottom couple lines
                # if (
                #     not self.get_right_canrange_detected()
                #     and not self.get_left_canrange_detected()
                #     and self.shot_timer.get()
                #     > self.canrange_debouncer.getDebounceTime() + 0.25
                # ):
                #     self.state = ShooterStates.COOLDOWN
                #     self.shot_timer.stop()
                #     self.shot_timer.reset()

            case ShooterStates.EJECT:
                self.io.target_indexer1_rps = -50
                self.io.target_indexer2_rps = -30
                self.io.target_rpm = 0
            case ShooterStates.COOLDOWN:
                self.io.target_indexer1_rps = 0.0
                self.io.target_indexer2_rps = 30

                if not self.shot_timer.isRunning():
                    self.shot_timer.start()

                if self.shot_timer.get() > 1:
                    self.io.target_rpm = 0.0
                    self.io.hood_zeroed = False
                    self.state = ShooterStates.IDLE

                    self.shot_timer.stop()
                    self.shot_timer.reset()

    def _set_up_logging(self) -> None:
        # self.io.add_function(self.get_average_rps)
        self.io.add_function(self.get_flywheel_speed_rpm)
        self.io.add_function(self.get_flywheel_speed_filtered)
        # self.io.add_function(self.get_indexer1_velocity)
        # self.io.add_function(self.get_indexer2_velocity)

        self.io.add_function(self.at_target_speed)
        self.io.add_function(self.get_stator_current)
        self.io.add_function(self.get_supply_current)
        self.io.add_function(self.get_supply_voltage)
        # self.io.add_function(self.get_absolute_hood_position, math.degrees)
        self.io.add_function(self.get_hood_angle, math.degrees)
        # self.io.add_function(self.get_hood_angle, alternate_name="hood position rad")

        self.io.add_function(
            lambda: self.get_supply_current() * self.get_supply_voltage(),
            alternate_name="Avg Shooter Power Draw",
        )

        self.io.add_function(
            lambda: calculate_power(
                self.io.left_bottom_supply_current_supplier,
                self.io.left_bottom_supply_voltage_supplier,
            ),
            alternate_name="Left Power Draw",
        )
        self.io.add_function(
            lambda: calculate_power(
                self.io.left_top_supply_current_supplier,
                self.io.left_top_supply_voltage_supplier,
            ),
            alternate_name="Middle Power Draw",
        )
        self.io.add_function(
            lambda: calculate_power(
                self.io.right_bottom_supply_current_supplier,
                self.io.right_bottom_supply_voltage_supplier,
            ),
            alternate_name="Right Power Draw",
        )

        self.io.add_function(
            lambda: calculate_power(
                self.io.feeder_supply_current_supplier,
                self.io.feeder_supply_voltage_supplier,
            ),
            alternate_name="Feeder Power Draw",
        )
        self.io.add_function(
            lambda: calculate_power(
                self.io.hood_supply_current_supplier,
                self.io.hood_supply_voltage_supplier,
            ),
            alternate_name="Hood Power Draw",
        )

        self.io.add_function(self.is_dry_firing)

    def _send_tuning_sendables(self) -> None:
        SmartDashboard.putData("Hood PID", self.hood_pid)
