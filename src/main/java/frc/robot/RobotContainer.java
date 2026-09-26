// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperStructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ShiftScheduler;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Serializer serializer;
  private final Shooter shooter;
  private final Intake intake;
  private Vision vision;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController auxController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        serializer = new Serializer();
        intake = new Intake();
        shooter = new Shooter();

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        serializer = new Serializer();
        intake = new Intake();
        shooter = new Shooter();
        vision = new Vision(drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        serializer = new Serializer();
        intake = new Intake();
        shooter = new Shooter();
        break;
    }

    vision = new Vision(drive);
    vision.setup();
    shooter.init();
    serializer.init();
    intake.init();

    // Register PathPlanner named commands for SuperStructure and setup Elastic tabs
    SuperStructure.registerNamedCommands(drive, shooter, intake, serializer);
    SuperStructure.setupElasticTab(shooter, intake);
    ShiftScheduler.setupElasticTab();
    ShiftScheduler.setDriverController(controller);
    ShiftScheduler.setAuxController(auxController);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default commands: idle states when no buttons/commands are active
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * (controller.leftBumper().getAsBoolean() ? 0.2 : 1.0),
            () -> -controller.getLeftX() * (controller.leftBumper().getAsBoolean() ? 0.2 : 1.0),
            () -> -controller.getRightX() * (controller.leftBumper().getAsBoolean() ? 0.2 : 1.0)));

    intake.setDefaultCommand(Commands.run(intake::resetIntake, intake));

    shooter.setDefaultCommand(
        Commands.run(
            () -> {
              shooter.stop();
              shooter.resetHood();
            },
            shooter));

    serializer.setDefaultCommand(Commands.run(serializer::stop, serializer));

    // ==========================================
    // DRIVER CONTROLLER BINDINGS (Port 0)
    // ==========================================

    // Driver Right Bumper: Intake Fuel
    controller
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.intakeStart(intake));

    // Driver Left Trigger: Snake Drive (0.8 speed) + Intake Fuel
    controller
        .leftTrigger()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(
            Commands.parallel(
                DriveCommands.snakeDrive(
                    drive, () -> -controller.getLeftY() * 0.8, () -> -controller.getLeftX() * 0.8),
                SuperStructure.intakeStart(intake)));

    // Driver Right Trigger: Smart Shoot / Pass on the Move with Aux Intake/Stash overrides
    controller
        .rightTrigger()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(
            Commands.parallel(
                DriveCommands.smartShoot(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                SuperStructure.smartShoot(
                    drive,
                    shooter,
                    intake,
                    serializer,
                    () ->
                        auxController.rightBumper().getAsBoolean()
                            || auxController.x().getAsBoolean(),
                    () ->
                        controller.rightBumper().getAsBoolean() || controller.y().getAsBoolean())));

    // Driver A Button: Stash Intake (held)
    controller
        .a()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.stashIntake(intake));

    // Driver X Button: Eject Fuel
    controller
        .x()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.ejectFuel(shooter, intake, serializer));

    // Driver B Button: Unjam
    controller
        .b()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.unjam(shooter, intake, serializer));

    // Driver Start Button: Emergency Stop All Superstructure
    controller
        .start()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.stopAll(shooter, intake, serializer));

    // Driver Back Button: Zero Gyro Heading
    controller.back().and(DriverStation::isTeleopEnabled).onTrue(DriveCommands.zeroGyro(drive));

    // ==========================================
    // AUX CONTROLLER BINDINGS (Port 1)
    // ==========================================

    // Aux Right Bumper: Stash Intake (also active while shooting!)
    auxController
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.stashIntake(intake));

    // Aux X Button: Emergency Stash Intake
    auxController
        .x()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.stashIntake(intake));

    // Aux Right Trigger: Warm Up Shooter Flywheel
    auxController
        .rightTrigger(0.2)
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(frc.robot.commands.ShooterCommands.spinUpDrum(shooter));

    // Aux Left Trigger: Eject / Unjam Fuel
    auxController
        .leftTrigger(0.2)
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.ejectFuel(shooter, intake, serializer));

    // Aux Left Bumper: Operator Cancel / Stop All
    auxController
        .leftBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(SuperStructure.stopAll(shooter, intake, serializer));

    // Aux Y Button: Increase Manual Shooter RPM Offset (+20 RPM)
    auxController
        .y()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> shooter.adjustManualRpmOffset(20.0), shooter));

    // Aux A Button: Decrease Manual Shooter RPM Offset (-20 RPM)
    auxController
        .a()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> shooter.adjustManualRpmOffset(-20.0), shooter));

    // Aux POV Up (D-Pad Up): Increase Manual Hood Angle Offset (+0.5 deg)
    auxController
        .povUp()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(
            Commands.runOnce(() -> shooter.adjustManualHoodOffset(Math.toRadians(0.5)), shooter));

    // Aux POV Down (D-Pad Down): Decrease Manual Hood Angle Offset (-0.5 deg)
    auxController
        .povDown()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(
            Commands.runOnce(() -> shooter.adjustManualHoodOffset(Math.toRadians(-0.5)), shooter));

    // Aux POV Right (D-Pad Right): Manual Intake Pivot Up (+5 deg)
    auxController
        .povRight()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> intake.adjustPivotAngle(Math.toRadians(5.0)), intake));

    // Aux POV Left (D-Pad Left): Manual Intake Pivot Down (-5 deg)
    auxController
        .povLeft()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> intake.adjustPivotAngle(Math.toRadians(-5.0)), intake));

    // Aux Start Button: Reset Intake Pivot Encoder Position to 0
    auxController
        .start()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(intake::resetPivotEncoder, intake));

    // Dashboard Controls
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "Zero Gyro", DriveCommands.zeroGyro(drive));
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "Reset Pose (0,0)", DriveCommands.resetPoseToZero(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
