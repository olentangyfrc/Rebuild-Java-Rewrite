import math

from components.intake import Intake
from components.shooter import Shooter
from components.serializer import Serializer
from components.drivetrain import Drivetrain
from components.climber import Climber
from wpilib import Timer
from wpimath import units

from components.serializer import Serializer
from components.shooter import Shooter
from components.intake import Intake
from utilities.states import (
    ShooterStates,
    SuperStructureStates,
    IntakeStates,
    ClimberStates,
)
from utilities.rebuilt_helpers import (
    FIELD_LENGTH,
    Interpolation,
    GamePositions,
    get_pass_distance,
)

from utilities.elasticlib import NotificationManager, Notification, NotificationLevel
from utilities.energy_profiles import RobotEnergyProfile
from magicbot import tunable


class SuperStructure:
    intake: Intake
    shooter: Shooter
    serializer: Serializer
    drivetrain: Drivetrain
    # climber: Climber

    def __init__(self) -> None:
        self._state = SuperStructureStates.IDLE
        self.timer = Timer()
        self.shoot_timer = Timer()
        self.interpolation = Interpolation()
        self.shooter_rpm = 1750

        self.hopper_full_wait_time = 1.25
        self.hopper_half_wait_time = 0.01
        self.active_wait_time = self.hopper_half_wait_time

        self.manual_hood_offset = 0.0
        self.manual_rpm_offset = 0.0

        self._emergency_stash = False

        self._use_agitate = False
        self._passing = False

    def toggle_hopper_full(self) -> None:
        """Toggles the hopper speed state."""
        self.hopper_is_half_full = not self.hopper_is_half_full

    @property
    def state(self) -> SuperStructureStates:
        return self._state

    @state.setter
    def state(self, new_state: SuperStructureStates) -> None:
        self._state = new_state

    def stash(self, emergency: bool = False) -> None:
        self._emergency_stash = emergency
        self.state = SuperStructureStates.STASH

    def cancel(self) -> None:
        self.state = SuperStructureStates.IDLE

    def intake_fuel(self) -> None:
        self.state = SuperStructureStates.INTAKE

    # def deploy_climber(self) -> None:
    #     self.state = SuperStructureStates.DEPLOYED_CLIMBER

    # def climb(self) -> None:
    #     self.state = SuperStructureStates.CLIMB

    def shoot_fuel(
        self,
        distance: units.meters,
        use_agitate: bool = True,
        agitate_timer: float = 1.5,
    ) -> None:
        self._use_agitate = use_agitate
        interp_data = self.interpolation.get_interpolated_values(distance)

        if interp_data is not None:
            angle, rpm = interp_data

            self.shooter.set_shoot_angle(angle + self.manual_hood_offset)
            self.shooter.set_shoot_rpm(rpm + self.manual_rpm_offset)

            self.active_wait_time = agitate_timer

            self.state = SuperStructureStates.SHOOT
            self._passing = False

        else:
            self.state = SuperStructureStates.IDLE
            NotificationManager.add_unconditional_notification(
                Notification(
                    title="Could not find interpolation data",
                    description="There is an issue with interpolating at this point. Superstructure will go back to idle.",
                )
            )

    def pass_fuel(self) -> None:
        distance = get_pass_distance(self.drivetrain.get_future_pose())

        self.shooter.set_shoot_angle(
            math.radians(0.39788 * (distance**2) - 4.2358 * distance + 41.14286)
        )
        self.shooter.set_shoot_rpm(
            30.79247 * (distance**2) - 268.77109 * distance + 2185.71429
        )

        self.state = SuperStructureStates.SHOOT
        self._use_agitate = False
        self._passing = True

    def apply_energy_profile(self, energy_profile: RobotEnergyProfile) -> None:
        self.intake.apply_current_limtis(energy_profile.intake)
        self.serializer.apply_current_limits(energy_profile.serializer)
        self.shooter.apply_current_limits(
            energy_profile.shooter, energy_profile.indexer1, energy_profile.indexer2
        )

    def unjam(self) -> None:
        self.state = SuperStructureStates.UNJAM

    def eject_fuel(self) -> None:
        self.state = SuperStructureStates.EJECT

    def execute(self) -> None:
        match self.state:
            case SuperStructureStates.IDLE:
                if self.shooter.state == ShooterStates.SHOOT:
                    self.shooter.cooldown()
                elif self.shooter.state != ShooterStates.COOLDOWN:
                    self.shooter.go_to_idle()

                self.intake.deploy()
                # self.climber.go_to_idle()
                self.serializer.go_to_idle()

                self.manual_hood_offset = 0
                self.manual_rpm_offset = 0

            case SuperStructureStates.STASH:
                if not self._emergency_stash:
                    self.intake.stash()
                else:
                    self.intake.emergency_stash()
                self.serializer.go_to_idle()
                self.shooter.go_to_idle()

            case SuperStructureStates.INTAKE:
                self.intake.intake()
                self.serializer.go_to_idle()
                self.shooter.go_to_idle()

            case SuperStructureStates.SHOOT:
                if self.shooter.state not in [
                    ShooterStates.SPIN_UP,
                    ShooterStates.SHOOT,
                ]:
                    self.shooter.shoot()

                    # if self.passing:
                    #     self.intake.intake()  # TODO: Find a better way to do this. You need to intake when passing, that's why this exists.
                    # else:
                    self.intake.deploy()
                    self.serializer.slow_reverse()

                    self.timer.stop()
                    self.timer.reset()

                if self.shooter.state == ShooterStates.SHOOT:
                    # self.intake.intake()
                    self.timer.start()

                    if self._passing:
                        self.intake.intake_pass()
                        self.serializer.pass_forward()
                    else:
                        if not self.intake.state in [
                            IntakeStates.AGITATE,
                            IntakeStates.STASHED,
                        ]:
                            self.serializer.forward()

                            if not self.intake.manual_override_flag:
                                self.intake.agitate()
                else:  # why should we do that?
                    self.timer.stop()
                    self.timer.reset()

            case SuperStructureStates.UNJAM:
                self.serializer.reverse()
                self.shooter.eject()

            case SuperStructureStates.EJECT:
                self.serializer.reverse()
                self.intake.eject()

            # case SuperStructureStates.DEPLOYED_CLIMBER:
            #     self.intake.stash()
            #     self.serializer.go_to_idle()
            #     self.climber.release()
            #     self.shooter.go_to_idle()

            # case SuperStructureStates.CLIMB:
            #     if not self.climber.state in [ClimberStates.HOLD, ClimberStates.CLIMB]:
            #         self.climber.climb()
            #     self.intake.retract()
            #     self.serializer.go_to_idle()
            #     self.shooter.go_to_idle()
