from magicbot import MagicRobot
import math
from ntcore import NetworkTableInstance
from phoenix6 import CANBus, SignalLogger
from phoenix6.configs import Slot0Configs
from phoenix6.signals import StaticFeedforwardSignValue
from wpilib import (
    Color8Bit,
    DataLogManager,
    DriverStation,
    Mechanism2d,
    RobotController,
    SmartDashboard,
    Timer,
    XboxController,
    getDeployDirectory,
)
from wpilib.deployinfo import getDeployData
from wpimath.geometry import Pose2d, Rotation2d, Transform3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory
from wpinet import WebServer


from components.drivetrain import DriveSignal, Drivetrain
from components.intake import Intake
from components.modules.generic_talon_fx_module import GenericTalonFXModule
from components.modules.onboard_talon_fx_module import OnboardTalonFxModule
from components.serializer import Serializer
from components.shooter import Shooter
from components.superstructure import SuperStructure
from components.vision import Vision
from utilities import helpers as utils
from utilities.configs import (
    DrivetrainConfig,
    IntakeConfig,
    SerializerConfig,
    ShooterConfig,
    SwerveConfig,
    VisionConfig,
    ClimberConfig,
)
from utilities.elasticlib import (
    Notification,
    NotificationLevel,
    NotificationManager,
    select_tab,
)
from utilities.energy_profiles import RobotEnergyProfile, EnergyProfiles
from utilities.helpers import (
    FIELD_LENGTH,
    FIELD_WIDTH,
    FFConstants,
    MotorTypes,
    PIDConstants,
    ProfileConstants,
    get_angle_between_poses,
    CurrentConfigs,
    within_rotation_tolerance,
)
from utilities.states import (
    DrivetrainStates,
    SuperStructureStates,
    IntakeStates,
    ClimberStates,
)
from utilities.IO import RobotIO, VisualizerIO
from utilities.IO_helpers import IO
from utilities.IO import RobotIO
from utilities.configs import (
    DrivetrainConfig,
    SwerveConfig,
    VisionConfig,
    ShooterConfig,
    IntakeConfig,
    SerializerConfig,
)
from utilities.IO_helpers import IO
from utilities.rebuilt_helpers import (
    GamePositions,
    ShiftScheduler,
    SoftWalls,
    get_pass_angle,
)

# WORKAROUND: Prevent CANrange from crashing LiveWindow due to strict WPILib bindings
from phoenix6.hardware.canrange import CANrange

CANrange.initSendable = lambda self, builder: None

from magicbot import tunable
# from components.climber import Climber


class MyRobot(MagicRobot):
    drivetrain: Drivetrain
    shooter: Shooter
    vision: Vision
    intake: Intake
    serializer: Serializer
    # climber: Climber

    superstructure: SuperStructure
    visualizer_io: VisualizerIO

    shoot_without_aligning = tunable(False)
    reset_to_mid_field = tunable(False)
    reset_to_full_field = tunable(False)

    brownout = tunable(False)
    speed = tunable(0.0)

    def createObjects(self) -> None:
        SoftWalls.TRENCH.value.enable()
        SoftWalls.SLOW_ZONE.value.disable()

        if self.isReal():
            DataLogManager.start()
            DataLogManager.logNetworkTables(True)
            DataLogManager.logConsoleOutput(True)
            DriverStation.startDataLog(DataLogManager.getLog())

        # Logs some meta data for advantagescope
        meta_table = NetworkTableInstance.getDefault().getTable("Metadata")
        deploy_info = getDeployData()

        if deploy_info is not None:
            for key, value in deploy_info.items():
                meta_table.putString(key, value)

        meta_table.putString("Runtime Type", self.getRuntimeType().name[1:])
        meta_table.putString("Serial Number", RobotController.getSerialNumber())

        self.main_controller = XboxController(0)
        self.aux_controller = XboxController(1)

        self.CANbus = CANBus("can0")

        swerve_config = SwerveConfig(
            drive_ratio=1 / 5.625,
            steer_ratio=1 / 25.9,
            steer_pid_constants=(
                Slot0Configs()
                .with_k_p(100 if self.isReal() else 50)
                .with_k_i(0)
                .with_k_d(0.5)
                .with_k_s(0.1)
                .with_k_v(2.48)
                .with_k_a(0)
                .with_static_feedforward_sign(
                    StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
                )
            ),
            drive_pid_constants=(
                Slot0Configs()
                .with_k_p(0.1 if self.isReal() else 0)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0)
                .with_k_v(0.124 if self.isReal() else 0)
            ),
            ff_constants=FFConstants(0.273, 6, 0),
            wheel_radius=0.08592 / 2,
            drive_motor_type=MotorTypes.KRAKEN_X60_FOC,
        )

        self.drivetrain_config = DrivetrainConfig(
            # Order of IDs is FL,FR,BL,BR
            drive_motor_ids=(10, 11, 12, 13),
            steer_motor_ids=(15, 16, 17, 18),
            steer_encoder_ids=(40, 41, 42, 43),
            gyro_id=5,
            # module_offsets=(-3.3, 9.15, 25.7, -82.17),
            module_offsets=(0.011, -0.0246, -0.0659, 0.228),
            drive_inverted=(False, False, False, False),  # True = clockwise positive
            steer_inverted=(False, False, False, False),
            encoder_inverted=(True, True, True, True),
            # this needs to be corret for this to work
            max_translation_speed=5.25,  # meters per second
            max_rotation_speed=12.2,  # radians per second
            enable_discretization=False,
            enable_passive_snap=False,
            automatically_lock=False,
            wheel_base=0.553,  # meters
            track_width=0.553,  # meters
            rotation_pid=PIDConstants(3, 0, 0.06),
            translation_pid=PIDConstants(3.2, 0, 0.12),
            auto_pid=PIDConstants(3.2, 0, 0.12),
            fast_snap_pid=PIDConstants(6.0, 0, 0.02),
            snake_pid=PIDConstants(5.3, 0, 0.05),
            module_type=OnboardTalonFxModule,
            swerve_config=swerve_config,
            CANbus=self.CANbus,
        )

        self.shooter_config = ShooterConfig(
            # Order of IDs is Left, Middle, Right, Need to find fourth motor
            shooter_motor_ids=(
                20,
                21,
                22,
                23,
            ),  # Right Top, Right Bottom, Left Top, Left Bottom
            indexer_id=(24, 25),
            hood_id=26,
            hood_encoder_id=47,
            # left_canrange=45,
            # right_canrange=46,
            hood_gear_ratio=1,
            hood_offset=-32,  # degrees
            shooter_gear_ratio=1,
            shoot_ff_constants=FFConstants(kS=0.16149, kV=0.13707, kA=0.042849),
            shoot_pid_constants=PIDConstants(p=0.21063),
            indexer1_ff_constants=FFConstants(kS=0.37921, kV=0.097873, kA=0.0029301),
            indexer1_pid_constants=PIDConstants(p=0.10865),
            indexer2_ff_constants=FFConstants(kS=0.34955, kV=0.09769, kA=0.0029558),
            indexer2_pid_constants=PIDConstants(p=0.17464),
            hood_ff_constants=FFConstants(kS=0.04, kG=0.29, kV=0),
            hood_pid_constants=PIDConstants(p=5.2, i=0, d=0),
            CANbus=self.CANbus,
        )
        # need to redo pid constants
        self.vision_config = VisionConfig(
            camera_names=("limelight-shooter", "limelight-left", "limelight-right"),
            robot_to_camera_transformations=(Transform3d(),),
        )

        self.intake_config = IntakeConfig(
            intake_roller_id=(28, 29),  # right, left
            intake_roller_ff_constants=FFConstants(
                kS=0.14158,
                kA=0.0038129,
                kV=0.12061,
            ),
            intake_roller_pid_constants=PIDConstants(p=0.21164 * 2),
            pivot_motor_id=30,
            pivot_encoder_id=51,
            pivot_gear_ratio=1 / 2.25,
            encoder_offset=47 - 9 - 14 + 1.1,
            pivot_ff_constants=FFConstants(kS=0.1, kG=0.35),
            pivot_pid_constants=PIDConstants(p=4, i=0, d=0.1),
            pivot_profile_constants=ProfileConstants(500, 250),
            CANbus=self.CANbus,
        )

        self.serializer_config = SerializerConfig(
            serializer_roller_id=32,
            serializer_ff_constants=FFConstants(kS=0.33796, kV=0.12192, kA=0.0029284),
            serializer_pid_constants=PIDConstants(p=0.10721),
            CANBus=self.CANbus,
        )

        # self.climber_config = ClimberConfig(
        #     climber_motor_id=31,
        #     climber_encoder_id=49,
        #     climber_ff_constants=FFConstants(kG=0.15, kS=0.25),
        #     climber_pid_constants=PIDConstants(p=110),
        #     CANBus=self.CANbus
        # )

        self.mech = Mechanism2d(4, 4, Color8Bit(255, 255, 255))
        self.intake_mech_root = self.mech.getRoot("Intake", 0, 0)
        self.shooter_mech_root = self.mech.getRoot("Shooter", 3, 2)

        SmartDashboard.putData("Mechanism", self.mech)

        self.timer = Timer()
        self.io = RobotIO()
        self.visualizer_io = VisualizerIO()
        self._set_up_notifications()

        # SignalLogger.stop()
        WebServer.getInstance().start(5800, getDeployDirectory())

        self.io.add_function(ShiftScheduler.get_active_alliance)
        self.io.add_function(ShiftScheduler.get_time_to_next_shift)
        self.io.add_function(ShiftScheduler.hub_is_active)
        self.io.add_function(
            lambda: self.main_controller.getLeftBumper(),
            alternate_name="Main Controller Connected",
        )
        self.io.add_function(
            lambda: self.aux_controller.getLeftBumper(),
            alternate_name="Aux Controller Connected",
        )

        self._driver_stash = False

        SignalLogger.enable_auto_logging(True)
        RobotController.setBrownoutVoltage(6)

    def robotPeriodic(self) -> None:
        hood_rad = self.shooter.get_hood_angle() - 0.35
        intake_rad = self.intake.get_pivot_angle() - 0.09
        self.visualizer_io.update(hood_rad, intake_rad)

        # Stops unimportant notifications during comp
        IO.update_publishers()
        self.watchdog.addEpoch("I/O")

        self.io.state = self.superstructure.state.name
        self.io.voltage = DriverStation.getBatteryVoltage()

        NotificationManager.send_notifications(DriverStation.isFMSAttached())
        self.watchdog.addEpoch("Notification Sending")

        if ShiftScheduler.get_current_shift() != self.io.shift:
            self.io.shift = ShiftScheduler.get_current_shift()

        self.io.time = self.timer.getMatchTime()
        self.io.aligned = self.drivetrain.aligned_callable()

        SmartDashboard.updateValues()

    def disabledInit(self) -> None:
        self.apply_energy_profile(EnergyProfiles.DEFAULT)

        self.superstructure.cancel()
        self.drivetrain.stop()
        IO.flush_publishers()  # Have to do here because publishers aren't setup otherwise
        self.vision.slow_down_processing()
        self.vision.capture_rewind()

        self.io.add_function(
            lambda: self._automodes.chooser.getSelected() is not None,
            alternate_name="Auto Selected",
        )

        self.main_controller.setRumble(XboxController.RumbleType.kBothRumble, 0)

        self.aux_controller.setRumble(XboxController.RumbleType.kBothRumble, 0)

        if DriverStation.isFMSAttached():
            select_tab("Preflight")
        else:
            select_tab("Systems")

    def autonomousInit(self):
        self.set_up_vision()
        select_tab("Autonomous")

    def teleopInit(self):
        select_tab("Teleoperated")

        self.superstructure.cancel()
        self.drivetrain.stop()
        self.set_up_vision()
        # self.vision.capture_rewind(30)
        self.drivetrain.drivetrain_field.getObject("traj").setPoses([])

    def teleopPeriodic(self) -> None:
        if self.brownout:
            self.apply_energy_profile(EnergyProfiles.BROWNOUT)
        elif self.io.energy_profile == EnergyProfiles.BROWNOUT.name:
            self.apply_energy_profile(EnergyProfiles.DEFAULT)

        if not DriverStation.isFMSAttached() and self.reset_to_mid_field:
            self.drivetrain.reset_pose(Pose2d(7.5, 0.71, 0))
        elif not DriverStation.isFMSAttached() and self.reset_to_full_field:
            self.drivetrain.reset_pose(Pose2d(14.1, 0.71, 0))

        if self.drivetrain.state == DrivetrainStates.LOCK and (
            not math.isclose(self.drivetrain.signal.get_magnitude(), 0.0)
            or not math.isclose(self.drivetrain.signal.speed.omega, 0.0)
        ):
            self.drivetrain.state = None

        if not ShiftScheduler.initialized:
            ShiftScheduler.schedule_shift_times()

        if ShiftScheduler.check_active_shift_upcoming(2):
            self.main_controller.setRumble(XboxController.RumbleType.kBothRumble, 0)
        elif ShiftScheduler.check_active_shift_upcoming(3):
            self.main_controller.setRumble(XboxController.RumbleType.kBothRumble, 0.60)

        if ShiftScheduler.check_active_shift_upcoming(5.5):
            self.aux_controller.setRumble(XboxController.RumbleType.kBothRumble, 0)
        elif ShiftScheduler.check_active_shift_upcoming(7):
            self.aux_controller.setRumble(XboxController.RumbleType.kBothRumble, 1)

        with self.consumeExceptions():
            self._drive_with_joystick()

        if self.main_controller.getLeftBumper() > 0.4:
            self.drivetrain.enable_motion_limiting(0.2, 1.0)
        else:
            self.drivetrain.disable_motion_limiting()

        if self.main_controller.getAButtonPressed():
            if self.superstructure.state == SuperStructureStates.STASH:
                self.superstructure.cancel()
                self._driver_stash = False
            else:
                self.superstructure.stash()
                self._driver_stash = True

        if self.main_controller.getRightTriggerAxis() > 0.4:
            if self.drivetrain.get_pose().X() < 5.2:
                self.drivetrain.enable_mt1_only()
                # What our shot rotation needs to be based on position
                shot_rotation = self.drivetrain.get_angle_to_hub()
                shot_dist = self.drivetrain.get_distance_to_hub()

                self.superstructure.shoot_fuel(shot_dist)

                if DriverStation.isFMSAttached() or not self.shoot_without_aligning:
                    if self.drivetrain.state != DrivetrainStates.LOCK:
                        # original rotate
                        self.drivetrain.go_to_rotation(Rotation2d(shot_rotation))

                # Bandaid fix
                self.drivetrain.aligned_callable = lambda: self.drivetrain.is_aligned(
                    translation_tolerance=None, rotation_tolerance=1, rotation_only=True
                )

                # Big jump in shot rotation from target rotation
                if (
                    self.aux_controller.getBButton()
                    or not abs(
                        shot_dist
                        * math.sin(
                            self.drivetrain.target_rotation.radians() - shot_rotation
                        )
                    )
                    < 0.2
                ):  # shot max translation error
                    self.drivetrain.go_to_rotation(Rotation2d(shot_rotation))

                # Original lock
                elif self.drivetrain.aligned_callable() or self.shoot_without_aligning:
                    if math.isclose(self.drivetrain.signal.get_magnitude(), 0):
                        self.drivetrain.lock()

                self.apply_energy_profile(EnergyProfiles.SHOOT)
            elif self.drivetrain.get_pose().X() < 11.3:
                if not self.shoot_without_aligning:
                    if math.isclose(self.drivetrain.signal.speed.omega, 0, abs_tol=0.1):
                        self.drivetrain.go_to_rotation(
                            get_pass_angle(self.drivetrain.get_pose())
                        )
                self.superstructure.pass_fuel()
                self.apply_energy_profile(EnergyProfiles.MID_FIELD_PASS)

                self.drivetrain.enable_motion_limiting(0.4, 1)
            else:
                if DriverStation.isFMSAttached() or not self.shoot_without_aligning:
                    if math.isclose(self.drivetrain.signal.speed.omega, 0, abs_tol=0.1):
                        self.drivetrain.go_to_rotation(Rotation2d(0))
                self.superstructure.pass_fuel()
                self.apply_energy_profile(EnergyProfiles.FAR_FIELD_PASS)
                self.drivetrain.enable_motion_limiting(0.4, 1)
        elif self.aux_controller.getRightTriggerAxis() > 0.2:
            self.shooter.warm_up_shooter()
        else:
            if self.drivetrain.state == DrivetrainStates.LOCK:
                self.drivetrain.state = None

            if self.superstructure.state == SuperStructureStates.SHOOT:
                self.drivetrain.disable_mt1_only()
                if self.shooter.state is not None:
                    self.superstructure.cancel()
                self.intake.agitation_timer.stop()
                self.drivetrain.state = None
                self.drivetrain.disable_motion_limiting()

                self.apply_energy_profile(EnergyProfiles.DEFAULT)

        if (
            self.main_controller.getYButton()
            and SoftWalls.TRENCH_ASSIST.value.contains(self.drivetrain.get_pose())
            and self.superstructure.state
            not in [SuperStructureStates.SHOOT, SuperStructureStates.PASS]
        ):
            rounded_rot = (
                round(self.drivetrain.get_pose().rotation().degrees() / 90) * 90
            )
            center_y = (
                0.56 if self.drivetrain.get_pose().Y() < 4 else FIELD_WIDTH - 0.56
            )

            if rounded_rot == 90:
                center_y -= 0.102
            elif rounded_rot == 270 or rounded_rot == -90:
                center_y += 0.102

            if not self.drivetrain.state == DrivetrainStates.DRIVER_ASSIST:
                self.drivetrain.activate_assists(
                    y_assist=True,
                    heading_assist=True,
                    target_y=center_y,
                    target_rot=rounded_rot,
                )
        elif self.drivetrain.state == DrivetrainStates.DRIVER_ASSIST:
            self.drivetrain.state = None

        if self.main_controller.getStartButton():
            self.superstructure.cancel()

        if self.main_controller.getBackButton():
            self.drivetrain.gyro.set_yaw(0)

        if self.aux_controller.getYButtonPressed():
            self.superstructure.manual_rpm_offset += 20

        if self.aux_controller.getAButtonPressed():
            self.superstructure.manual_rpm_offset -= 20

        pov = self.aux_controller.getPOV()
        if pov == 0:
            self.superstructure.manual_hood_offset += math.radians(0.5)
        elif pov == 180:
            self.superstructure.manual_hood_offset -= math.radians(0.5)
        if pov == 90:
            self.intake.manual_override(0.7)
            self.intake.io.target_pivot_angle += math.radians(5)
        elif pov == 270:
            self.intake.manual_override(-0.7)
            self.intake.io.target_pivot_angle -= math.radians(5)
        elif self.intake.manual_override_flag:
            self.intake.manual_override_flag = False

        if self.aux_controller.getRightBumper():
            self.intake.stash()
            self._driver_stash = False
        elif self.aux_controller.getXButton():
            self.superstructure.stash(emergency=True)
            self._driver_stash = False
        elif (
            self.superstructure.state == SuperStructureStates.STASH
            and not self._driver_stash
        ):
            self.superstructure.cancel()

        if self.aux_controller.getStartButton():
            self.intake.pivot_encoder.set_position(
                self.intake_config.encoder_offset / 360
            )

        if self.aux_controller.getBackButtonPressed():
            self.shooter.io.hood_zeroed = False

        if self.aux_controller.getLeftBumper():
            self.operator_cancel()

        if self.main_controller.getXButton():
            self.superstructure.eject_fuel()

        elif self.main_controller.getBButton():
            self.superstructure.unjam()
        elif self.aux_controller.getLeftTriggerAxis() > 0.2:
            if self.superstructure.state == SuperStructureStates.SHOOT:
                self.superstructure.unjam()
            else:
                self.superstructure.eject_fuel()
        elif self.superstructure.state in [
            SuperStructureStates.UNJAM,
            SuperStructureStates.EJECT,
        ]:
            self.superstructure.cancel()
            
        if self.main_controller.getRightBumper() > 0.4:
            self.superstructure.intake_fuel()
        elif self.main_controller.getLeftTriggerAxis() > 0.4:
            self.drivetrain.enable_motion_limiting(0.8, 1)
            if math.isclose(self.drivetrain.signal.speed.omega, 0, abs_tol=0.1):
                self.drivetrain.snake()
            self.superstructure.intake_fuel()
        elif self.superstructure.state == SuperStructureStates.INTAKE:
            if self.drivetrain.state == DrivetrainStates.SNAKE:
                self.drivetrain.state = None
            self.superstructure.cancel()
            self.drivetrain.disable_motion_limiting()

        # Update and run Soft Wall behaviors
        self._run_softwall_logic()

    def disabledPeriodic(self):
        if DriverStation.isFMSAttached():
            select_tab("Preflight")

    def testPeriodic(self):
        print("This is Tests")

    def set_up_vision(self) -> None:
        self.vision.mt2_nt_topic = (
            "botpose_orb_wpiblue"
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue
            else "botpose_orb_wpired"
        )
        self.vision.mt1_nt_topic = (
            "botpose_wpiblue"
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue
            else "botpose_wpired"
        )
        self.vision.speed_up_processing()

    def _drive_with_joystick(self) -> None:
        vx = (
            utils.filter_input(self.main_controller.getLeftY())
            * self.drivetrain_config.max_translation_speed
        )
        vy = (
            utils.filter_input(self.main_controller.getLeftX())
            * self.drivetrain_config.max_translation_speed
        )
        omega = (
            utils.filter_input(self.main_controller.getRightX())
            * self.drivetrain_config.max_rotation_speed
        )

        self.drivetrain.signal = DriveSignal(ChassisSpeeds(-vx, -vy, -omega))

    def _run_softwall_logic(self) -> None:
        #     """Handles specialized behaviors for different field zones."""

        if SoftWalls.TRENCH.value.contains(pose=self.drivetrain.get_pose()):
            self.shooter.lock_hood()
            SmartDashboard.putString("SoftWall Status", "TRENCH: Hood Inhibited")
        else:
            self.shooter.unlock_hood()
            SmartDashboard.putString("SoftWall Status", "Nothing")

    #     if SoftWalls.SLOW_ZONE.value.contains(pose):
    #         self.drivetrain.enable_motion_limiting(0.2, 1.0)
    #         SmartDashboard.putString("SoftWall Status", "PRECISION ZONE: Speed Limited")
    #     else:
    #         # SmartDashboard.putString("SoftWall Status","Nothing")
    #         self.drivetrain.disable_motion_limiting()
    #         # self.cancel_all()

    #     for softwall in SoftWalls:
    #         softwall.value.update_on_field(self.drivetrain.drivetrain_field)

    def _set_up_notifications(self) -> None:
        NotificationManager.add_conditional_notification(
            Notification(
                level=NotificationLevel.WARNING,
                title="Low Battery Voltage",
                description="Consider swapping soon",
            ),
            lambda: (
                (RobotController.getBatteryVoltage() < 8 and self.isReal())
                or (
                    RobotController.getBatteryVoltage() < 12.25
                    and self.isDisabled()
                    and self.isReal()
                )
                # The sims are harder on battery voltage
                or (RobotController.getBatteryVoltage() < 6 and self.isSimulation())
            ),
        )

        NotificationManager.add_conditional_notification(
            Notification(
                level=NotificationLevel.ERROR,
                title="Robot is browning out",
                description="Stop robot and replace battery",
            ),
            RobotController.isBrownedOut,
        )

        NotificationManager.add_conditional_notification(
            Notification(
                level=NotificationLevel.INFO,
                title="Active Shift Coming Up",
                description="Active hub in 5 seconds",
            ),
            ShiftScheduler.check_active_shift_upcoming,
        )

        NotificationManager.add_conditional_notification(
            Notification(
                level=NotificationLevel.WARNING,
                title="Current Auto is Set to None",
                description="Please Select and Auto",
            ),
            lambda: self._automodes.chooser.getSelected() is None,
            cooldown=3,
        )

    def operator_cancel(self) -> None:
        NotificationManager.add_unconditional_notification(
            Notification(title="Cancelling all commands")
        )
        self.superstructure.cancel()
        self.drivetrain.stop()
        self.drivetrain.operator_lock = False

        self.apply_energy_profile(EnergyProfiles.DEFAULT)

    def apply_energy_profile(self, energy_profile: EnergyProfiles):
        if self.io.energy_profile != energy_profile.name:
            self.drivetrain.apply_current_limits(energy_profile.value.drivetrain)
            self.superstructure.apply_energy_profile(energy_profile.value)
            self.io.energy_profile = energy_profile.name
