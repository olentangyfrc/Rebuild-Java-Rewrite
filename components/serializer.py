from magicbot import tunable
from phoenix6 import BaseStatusSignal
from phoenix6.controls import VoltageOut, MotionMagicVelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.configs import CurrentLimitsConfigs, TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue

from utilities.configs import SerializerConfig
from utilities.helpers import calculate_power
from utilities.states import SerializerStates
from utilities.IO import SerializerIO

# can id is 27


class Serializer:
    manual_roller_floor_rps = 0.0
    manual = False

    def __init__(self, config: SerializerConfig):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor_configs.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.slot0__configs = self.motor_configs.slot0
        self.slot0__configs.k_s = config.serializer_ff_constants.kS
        self.slot0__configs.k_v = config.serializer_ff_constants.kV
        self.slot0__configs.k_a = config.serializer_ff_constants.kA
        self.slot0__configs.k_p = config.serializer_pid_constants.p
        self.slot0__configs.k_i = config.serializer_pid_constants.i
        self.slot0__configs.k_d = config.serializer_pid_constants.d
        self.motion_magic_configs = self.motor_configs.motion_magic

        self.motion_magic_configs.motion_magic_acceleration = 9999
        self.serializer_motor = TalonFX(config.serializer_roller_id, config.CANBus)
        self.serializer_motor.configurator.apply(self.motor_configs)

        self.config = config
        self.io = SerializerIO()

        self.io.serializer_rps_supplier = self.serializer_motor.get_velocity()

        self._state = SerializerStates.IDLE

        self.io.target_rps = 0.0

        self.io.serializer_supply_current_supplier = (
            self.serializer_motor.get_supply_current()
        )
        self.io.serializer_supply_voltage_supplier = (
            self.serializer_motor.get_supply_voltage()
        )

        self.io.serializer_stator_current_supplier = (
            self.serializer_motor.get_stator_current()
        )

        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.io.serializer_supply_current_supplier,
            self.io.serializer_supply_voltage_supplier,
            self.io.serializer_stator_current_supplier,
            self.io.serializer_rps_supplier,
        )

        self.serializer_voltage_request = VoltageOut(0.0)
        self.serializer_magic_motion_request = MotionMagicVelocityVoltage(0.0)

        self.io.add_function(
            lambda: calculate_power(
                self.io.serializer_supply_current_supplier,
                self.io.serializer_supply_voltage_supplier,
            ),
            alternate_name="Serializer Power Draw",
        )

    @property
    def state(self) -> SerializerStates:
        return self._state

    @state.setter
    def state(self, new_state: SerializerStates) -> None:
        if not (isinstance(new_state, SerializerStates) or new_state is None):
            raise ValueError("State must be a Serializer State")
        self._state = new_state

    def go_to_idle(self) -> None:
        self.state = SerializerStates.IDLE

    def forward(self) -> None:
        self.state = SerializerStates.FORWARD

    def reverse(self) -> None:
        self.state = SerializerStates.REVERSE

    def slow_reverse(self) -> None:
        self.state = SerializerStates.SLOW_REVERSE

    def pass_forward(self) -> None:
        self.state = SerializerStates.PASS_FORWARD

    def apply_current_limits(self, current_limts: CurrentLimitsConfigs):
        self.serializer_motor.configurator.apply(current_limts)

    def execute(self):
        match self.state:
            case SerializerStates.IDLE:
                self.io.target_rps = 0
            case SerializerStates.FORWARD:
                self.io.target_rps = 85
            case SerializerStates.REVERSE:
                self.io.target_rps = -85
            case SerializerStates.SLOW_REVERSE:
                self.io.target_rps = -20
            case SerializerStates.PASS_FORWARD:
                self.io.target_rps = 40

        self.io.state = self.state.name

        if self.manual:
            self.io.target_rps = self.manual_roller_floor_rps

        self.serializer_motor.set_control(
            self.serializer_magic_motion_request.with_velocity(
                self.io.target_rps
            ).with_enable_foc(True)
        )
