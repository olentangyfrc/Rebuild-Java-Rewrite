from pathlib import Path
import wpilib
import math
from wpimath.geometry import Pose2d, Rotation2d
import choreo
from choreo.trajectory import SwerveTrajectory, SwerveSample
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import RobotBase, DriverStation, SmartDashboard
from wpimath.kinematics import ChassisSpeeds
from components.intake import Intake
from components.superstructure import SuperStructure
from components.shooter import Shooter
from components.vision import Vision

from components.drivetrain import Drivetrain, DriveSignal
from utilities.elasticlib import select_tab
from utilities.helpers import (
    within_pose_tolerance,
    AutonActions,
    PathProperty,
    inputModulus,
    get_angle_between_poses,
)
from utilities.rebuilt_helpers import FIELD_LENGTH, get_pass_angle
from utilities.states import DrivetrainStates, SuperStructureStates
from utilities.energy_profiles import EnergyProfiles, RobotEnergyProfile

TEST_ON_BLOCKS = False  # Tghis Basically just ignore the "real" gyro and uses the trajectory's should be at point you do also have to chnage anot variable in drivetrain.py
base_dir = "/home/lvuser/py/deploy/choreo/"
if not RobotBase.isReal():
    base_dir = Path("deploy/choreo").resolve().as_posix() + "/"

traj_files = Path.glob(Path(base_dir), "*.traj")

TRAJECTORY_LOOKUP = {
    traj_file.stem: choreo.load_swerve_trajectory(str(traj_file.with_suffix("")))
    for traj_file in traj_files
}

trajectory_giveuptime = 0.5  # seconds


AUTO_MAX_SPEED = 5.83  # m/s


class BaseAutonomous(AutonomousStateMachine):
    _AutonomousStateMachine__engaged = True

    # Injecting the drivetrain component
    drivetrain: Drivetrain
    intake: Intake
    shooter: Shooter
    superstructure: SuperStructure
    vision: Vision

    CLIMB_DURATION = 4.0  # Time needed for the physical climb
    TOTAL_AUTO_TIME = 22.0
    STATIONARY_ACTION_TIME = 4.0  # How long to stay at destination if parallel=False

    PATH_CORRECTION_ENABLED = False
    PATH_CORRECTION_SEARCH_WINDOW = 1.0  # seconds
    PATH_CORRECTION_SAMPLES = 50
    DRIFT_CORRECTION_ENABLED = False
    DRIFT_TOLERANCE = 0.5  # meters

    TRANSLATION_TOLERANCE = 0.5  # meters
    ROTATION_TOLERANCE = 3.0  # degrees

    def __init__(
        self, paths: list[PathProperty] = [], manual_start_pose: Pose2d | None = None
    ) -> None:
        self.wait_timer = wpilib.Timer()
        self.path_timer = wpilib.Timer()
        self.action_timer = wpilib.Timer()

        self.paths = paths
        self.realign_target_time = 0.0
        self.path_start_offset = 0.0
        self.bump_target_x = 4.5
        self.bump_speed = -1.6

        self.manual_start_pose = manual_start_pose

        self.energy_profile = EnergyProfiles.DEFAULT
        self.apply_energy_profile(self.energy_profile)

        super().__init__()

    def apply_energy_profile(self, energy_profile: EnergyProfiles):
        if (
            energy_profile != self.energy_profile and RobotBase.isReal()
        ):  # Causes lag in sim
            self.drivetrain.apply_current_limits(energy_profile.value.drivetrain)
            self.superstructure.apply_energy_profile(energy_profile.value)
            self.energy_profile = energy_profile

    @state(first=True)
    def startup(self) -> None:
        self.locked_heading = None
        self.current_path_index = 0

        if not self.paths:
            self.done()
            return

        self.current_path = self.paths[self.current_path_index]

        initial_traj = TRAJECTORY_LOOKUP.get(self.current_path.path_name)

        if self.manual_start_pose is not None and initial_traj is not None:
            raise ValueError(
                "Manual Start Pose cannot be set if inital path is choreo trajectory"
            )

        if initial_traj is not None:
            initial_pose = initial_traj.get_initial_pose()

            if initial_pose is not None:
                if self.current_path.flipped:
                    initial_pose = Pose2d(
                        FIELD_LENGTH - initial_pose.X(),
                        initial_pose.Y(),
                        Rotation2d(
                            inputModulus(
                                math.pi - initial_pose.rotation().radians(),
                                0,
                                math.tau,
                            )
                        ),
                    )
                self.drivetrain.reset_pose(initial_pose)
        elif self.manual_start_pose is not None:
            self.drivetrain.reset_pose(self.manual_start_pose)
        else:
            self.done()

        self.wait_timer.reset()
        self.path_timer.reset()

        self.path_start_offset = 0.0

        self._handle_state_transitions()

    @state
    def follow_trajectory(self, initial_call: bool) -> None:
        self.drivetrain.state = None
        # 1. EMERGENCY CLIMB CHECK
        # time_left_in_match = self.TOTAL_AUTO_TIME - self.wait_timer.get()
        # if time_left_in_match <= self.CLIMB_DURATION:
        #     self.next_state("auto_climb")
        #     return

        if self.current_path_index >= len(self.paths):
            self.done()
            return

        if initial_call:
            if self.current_path.stashed:
                self.superstructure.stash()
            else:
                self.superstructure.cancel()

            if getattr(self.current_path, "parallel", True):
                self.handle_parallel_actions(self.current_path.auton_action)
            if not self.path_timer.isRunning():
                self.path_timer.start()
            if self.current_path.reverse and getattr(
                self.current_path, "lock_heading", True
            ):
                self.locked_heading = self.drivetrain.get_pose().rotation()
            else:
                self.locked_heading = None

            self.apply_energy_profile(EnergyProfiles.AUTO_DRIVE)

        self.drivetrain.aligned_callable = lambda: self.drivetrain.is_aligned(
            translation_tolerance=self.current_path.translation_tolerance
            or self.TRANSLATION_TOLERANCE,
            rotation_tolerance=self.current_path.rotation_tolerance
            or self.ROTATION_TOLERANCE,
        )

        trajectory = TRAJECTORY_LOOKUP.get(self.current_path.path_name)
        if trajectory is None:
            print(f"Could not load trajectory: {self.current_path.path_name}")
            self.current_path_index += 1
            self.path_timer.reset()
            self.next_state("follow_trajectory")
            return

        time_elapsed = self.path_timer.get() - self.path_start_offset
        # RELASE INTAKE EARLY TIMER
        # CHANGE THIS
        if self.current_path.stashed:
            if trajectory.get_total_time() - time_elapsed <= 1.0:
                self.superstructure.cancel()

        time = time_elapsed

        if self.PATH_CORRECTION_ENABLED:
            corrected_time, path_deviation = self.find_closest_point_time(trajectory)
            time = corrected_time
            if self.DRIFT_CORRECTION_ENABLED and path_deviation > self.DRIFT_TOLERANCE:
                self.realign_target_time = corrected_time
                self.next_state("realign_to_path")
                return

        target_pose = (
            trajectory.get_final_pose()
            if not self.current_path.reverse
            else trajectory.get_initial_pose()
        )

        if not target_pose:
            self.current_path_index += 1
            return

        self.drivetrain.target_pose = target_pose

        if self.current_path.reverse:
            time = trajectory.get_total_time() - time

        sample = trajectory.sample_at(time, False)

        if sample:
            augmented = self.augment_sample(sample, time, self.current_path.flipped)
            if self.current_path.reverse:
                # Negate velocities and accelerations for reverse travel
                rev_vx = -augmented.vx
                rev_vy = -augmented.vy
                rev_omega = -augmented.omega
                rev_ax = -augmented.ax
                rev_ay = -augmented.ay
                rev_alpha = -augmented.alpha

                if self.locked_heading is not None:
                    # Create a new sample with our locked heading
                    final_sample = SwerveSample(
                        augmented.timestamp,
                        augmented.x,
                        augmented.y,
                        self.locked_heading.radians(),
                        rev_vx,
                        rev_vy,
                        0.0,  # No angular velocity when heading is locked
                        rev_ax,
                        rev_ay,
                        0.0,  # No angular acceleration
                        augmented.fx,
                        augmented.fy,
                    )
                else:
                    # Create a new sample with a flipped heading
                    final_sample = SwerveSample(
                        augmented.timestamp,
                        augmented.x,
                        augmented.y,
                        inputModulus(augmented.heading + math.pi, 0, math.tau),
                        rev_vx,
                        rev_vy,
                        rev_omega,
                        rev_ax,
                        rev_ay,
                        rev_alpha,
                        augmented.fx,
                        augmented.fy,
                    )
            else:
                final_sample = augmented
            is_heading_locked = (
                self.locked_heading is not None and self.current_path.reverse
            )
            self.drivetrain.follow_trajectory(
                final_sample,
                trajectory,
                is_reversed=self.current_path.reverse,
                is_heading_locked=is_heading_locked,
                test_on_blocks=TEST_ON_BLOCKS,
                auto_speed_limit=AUTO_MAX_SPEED,
            )

        if self.current_path.continuous:
            path_is_finished = (
                self.path_timer.get() - self.path_start_offset
            ) >= trajectory.get_total_time()
        else:
            path_is_finished = self.drivetrain.aligned_callable() or (
                self.path_timer.get()
                - self.path_start_offset
                - trajectory.get_total_time()
                > trajectory_giveuptime
            )
        if (
            self.current_path.ignore_safety == "Known Point"
            and (self.path_timer.get() - self.path_start_offset)
            >= trajectory.get_total_time() * 0.5
        ):
            path_is_finished = True

        if path_is_finished:
            is_continuous_transition = (
                self.current_path.continuous
                and self.current_path_index < len(self.paths) - 1
            )
            if is_continuous_transition:
                self.path_start_offset += trajectory.get_total_time()
            else:
                self.path_timer.stop()
                self.path_timer.reset()
                self.path_start_offset = 0.0

            next_path_continues_intake = False
            if self.current_path_index + 1 < len(self.paths):
                next_path = self.paths[self.current_path_index + 1]
                if next_path.continue_intake:
                    next_path_continues_intake = True

            if not next_path_continues_intake:
                self.superstructure.cancel()
            self.current_path_index += 1
            self._handle_state_transitions()

    def handle_parallel_actions(self, action: AutonActions):
        """Hardware triggers for moving and acting simultaneously."""
        if action == AutonActions.SCORE:
            print("moving and shooting")
            # self.superstructure.shoot()

        elif action == AutonActions.INTAKE:
            self.superstructure.intake_fuel()

        if action == AutonActions.DEPLOY_INTAKE:
            self.superstructure.cancel()  # Cancel any current superstructure action to deploy intake

    @state
    def wait(self) -> None:
        self.drivetrain.stop()

        if self.current_path.stashed:
            self.intake.stash()

        if not self.wait_timer.isRunning():
            self.wait_timer.start()

        if self.current_path.stashed:
            self.intake.stash()

        if self.wait_timer.get() > self.current_path.wait:
            self.current_path_index += 1
            self._handle_state_transitions()

            self.wait_timer.stop()
            self.wait_timer.reset()

    @state
    def execute_stationary_action(self, initial_call):
        """Triggered at the end of a path for stopped actions."""
        current_path = self.paths[self.current_path_index]

        if initial_call:
            self.stop_drivetrain()
            self.action_timer.restart()
            print(
                f"Stationary Action Started at Destination: {current_path.auton_action}"
            )
            if current_path.auton_action == AutonActions.SCORE:
                self.superstructure.cancel()  # Cancel any current superstructure action to start shooting

        if current_path.auton_action == AutonActions.SCORE:
            self.apply_energy_profile(EnergyProfiles.SHOOT)

            self.drivetrain.aligned_callable = lambda: self.drivetrain.is_aligned(
                translation_tolerance=None, rotation_tolerance=2, rotation_only=True
            )

            if self.drivetrain.get_pose().X() < 4.5:
                self.superstructure.shoot_fuel(
                    self.drivetrain.get_distance_to_hub(),
                    use_agitate=current_path.use_agitate,
                    agitate_timer=current_path.agitate_delay,
                )
                self.drivetrain.go_to_rotation(
                    Rotation2d(self.drivetrain.get_angle_to_hub())
                )
                if self.drivetrain.aligned_callable():
                    if math.isclose(self.drivetrain.signal.get_magnitude(), 0):
                        self.drivetrain.lock()
            elif self.drivetrain.get_pose().X() < 11.3:
                self.drivetrain.go_to_rotation(
                    get_pass_angle(self.drivetrain.get_pose())
                )
                self.superstructure.pass_fuel()

        elif current_path.auton_action == AutonActions.INTAKE:
            self.superstructure.intake_fuel()

        action_time = self.STATIONARY_ACTION_TIME
        if current_path.stationary_action_time is not None:
            action_time = current_path.stationary_action_time

        if self.action_timer.get() > action_time:
            self.action_timer.stop()
            self.action_timer.reset()
            self.current_path_index += 1
            self._handle_state_transitions()

    @state
    def line_up(self, initial_call: bool) -> None:
        if initial_call:
            self.apply_energy_profile(EnergyProfiles.DEFAULT)

            if self.current_path.stashed:
                self.superstructure.stash()
            else:
                self.superstructure.cancel()

        if self.current_path.auton_action == AutonActions.INTAKE:
            self.superstructure.intake_fuel()

        if self.current_path.line_up_to is not None and (
            not self.drivetrain.state == DrivetrainStates.LINE_UP
            or self.drivetrain.target_pose != self.current_path.line_up_to
        ):
            self.drivetrain.go_to_pose(self.current_path.line_up_to)

        self.drivetrain.aligned_callable = lambda: self.drivetrain.is_aligned(
            translation_tolerance=0.15,
            rotation_tolerance=8,
            y_only=self.current_path.line_up_y_only,
        )

        if self.drivetrain.aligned_callable():
            self.current_path_index += 1
            self._handle_state_transitions()

    @state
    def snap_to_rotation(self) -> None:
        self.shooter.warm_up_shooter(1800)
        if self.current_path.snap_to_rotation is not None:
            self.drivetrain.go_to_rotation(self.current_path.snap_to_rotation)

        self.drivetrain.aligned_callable = lambda: self.drivetrain.is_aligned(
            translation_tolerance=None, rotation_tolerance=2, rotation_only=True
        )

        if self.drivetrain.aligned_callable():
            self.current_path_index += 1
            self._handle_state_transitions()

    @state
    def realign_to_path(self, initial_call: bool) -> None:
        trajectory = TRAJECTORY_LOOKUP.get(self.current_path.path_name)
        if trajectory is None:
            self.next_state("follow_trajectory")
            return

        target_sample = trajectory.sample_at(self.realign_target_time, False)
        target_pose = Pose2d(
            target_sample.x, target_sample.y, Rotation2d(target_sample.heading)
        )

        if within_pose_tolerance(self.drivetrain.get_pose(), target_pose, 0.5, 2.0):
            self.next_state("follow_trajectory")
            return

        drive_to_sample = SwerveSample(
            self.realign_target_time,
            target_pose.X(),
            target_pose.Y(),
            target_pose.rotation().radians(),
            0.0,
            0.0,
            0.0,  # Zero velocity
            0.0,
            0.0,
            0.0,  # Zero acceleration
            [0.0, 0.0, 0.0, 0.0],  # fx
            [0.0, 0.0, 0.0, 0.0],  # fy
        )

        self.drivetrain.follow_trajectory(
            drive_to_sample,
            trajectory,
            is_reversed=False,
            is_heading_locked=False,
        )

    @state
    def bump(self) -> None:
        """
        Drives along the X axis until a target X coordinate is reached.

        This state requires the following instance variables to be set before it is
        called:
        - self.bump_target_x: The target X coordinate.
        - self.bump_speed: The speed to drive at. Positive for +X, negative for -X.
        - self.bump_exit_state: The name of the state to transition to when finished.
                               Defaults to 'done'.
        """
        current_x = self.drivetrain.get_pose().X()
        self.superstructure.intake_fuel()
        self.shooter.warm_up_shooter(1800)

        finished = False
        if (
            self.bump_speed > 0
        ):  # Moving in positive X direction, stop when GREATER than
            if current_x > self.bump_target_x:
                finished = True
        elif self.bump_speed < 0:  # Moving in negative X direction, stop when LESS than
            if current_x < self.bump_target_x:
                finished = True
        else:  # Speed is 0, so we are done.
            finished = True

        if finished:
            self.drivetrain.stop()
            self.current_path_index += 1
            self._handle_state_transitions()
        else:
            self.drivetrain.signal = DriveSignal(
                ChassisSpeeds(self.bump_speed, 0, 0), field_relative=True
            )

    # @state
    # def auto_climb(self, initial_call):
    #     if initial_call:
    #         self.stop_drivetrain()
    #         print("!!! EMERGENCY CLIMB TRIGGERED !!!")
    #         print("lineing up to climb" * 10)
    #         self.action_timer.restart()

    #     if self.action_timer.get() > self.CLIMB_DURATION:
    #         print("Climb sequence complete.")
    #         self.done()

    def find_closest_point_time(
        self, trajectory: SwerveTrajectory
    ) -> tuple[float, float]:
        current_pose = self.drivetrain.get_pose()
        expected_time = self.path_timer.get()

        start_time = max(0.0, expected_time - self.PATH_CORRECTION_SEARCH_WINDOW / 2)
        end_time = min(
            trajectory.get_total_time(),
            expected_time + self.PATH_CORRECTION_SEARCH_WINDOW / 2,
        )

        if start_time >= end_time:  # at start or end of path
            sample = trajectory.sample_at(expected_time, False)
            sample_pose = Pose2d(sample.x, sample.y, Rotation2d(0))
            distance = current_pose.translation().distance(sample_pose.translation())
            return expected_time, distance

        time_step = (end_time - start_time) / self.PATH_CORRECTION_SAMPLES

        closest_time = expected_time
        min_distance = float("inf")

        for i in range(self.PATH_CORRECTION_SAMPLES + 1):
            t = start_time + i * time_step
            sample = trajectory.sample_at(t, False)
            sample_pose = Pose2d(sample.x, sample.y, Rotation2d(0))
            distance = current_pose.translation().distance(sample_pose.translation())

            if distance < min_distance:
                min_distance = distance
                closest_time = t

        return closest_time, min_distance

    def augment_sample(
        self, sample: SwerveSample, time: float, flipped: bool
    ) -> SwerveSample:
        aug = SwerveSample(
            time,
            sample.x,
            sample.y,
            sample.heading,
            sample.vx,
            sample.vy,
            sample.omega,
            sample.ax,
            sample.ay,
            sample.alpha,
            sample.fx,
            sample.fy,
        )
        if flipped:
            aug.x = FIELD_LENGTH - sample.x
            aug.heading = inputModulus(math.pi - sample.heading, 0, math.tau)
            aug.vx = -sample.vx
            aug.omega = -sample.omega
            aug.ax = -sample.ax
            aug.alpha = -sample.alpha
        return aug

    def stop_drivetrain(self) -> None:
        self.drivetrain.signal = DriveSignal(ChassisSpeeds(0, 0, 0))

    def done(self) -> None:
        self.stop_drivetrain()
        self.wait_timer.stop()
        self.path_timer.stop()

        cancel_superstructure = True

        if self.current_path_index >= len(self.paths) and len(self.paths) > 0:
            last_path = self.paths[-1]
            if (
                not last_path.parallel
                and last_path.auton_action != AutonActions.NOTHING
            ):
                cancel_superstructure = False

        if cancel_superstructure:
            self.superstructure.cancel()

        super().done()

    def _handle_state_transitions(self) -> None:
        if self.current_path_index >= len(self.paths):
            self.done()
            return

        self.current_path = self.paths[self.current_path_index]

        if self.current_path.path_name.upper() == "BUMP":
            if (
                self.current_path.bump_target_x is None
                or self.current_path.bump_speed is None
            ):
                print("ERROR: Bump path property is missing target_x or speed")
                self.current_path_index += 1
                self.next_state("follow_trajectory")
                return

            self.bump_target_x = self.current_path.bump_target_x
            self.bump_speed = self.current_path.bump_speed
            self.bump_exit_state = "follow_trajectory"
            self.next_state("bump")
            return
        elif self.current_path.path_name.upper() == "NOTHING":
            if (
                self.current_path.parallel is False
                and self.current_path.auton_action != AutonActions.NOTHING
            ):
                self.next_state("execute_stationary_action")
            elif self.current_path.wait > 0:
                self.next_state("wait")
            else:
                self.current_path_index += 1
                self.next_state("follow_trajectory")
            return
        elif self.current_path.path_name.upper() == "LINE_UP":
            if self.current_path.line_up_to is None:
                print("ERROR: Line up path property is missing a target pose")
                self.current_path_index += 1
                self.next_state("follow_trajectory")
                return

            self.next_state("line_up")
            return
        elif self.current_path.path_name.upper() == "SNAP_TO_ROTATION":
            if self.current_path.snap_to_rotation is None:
                print("ERROR: Snap to rotation path property is missing a target pose")
                self.current_path_index += 1
                self.next_state("follow_trajectory")
                return

            self.next_state("snap_to_rotation")
            return
        else:
            self.next_state("follow_trajectory")
