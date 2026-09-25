// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

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

    // Register PathPlanner named commands for SuperStructure and setup Elastic tab
    SuperStructure.registerNamedCommands(drive, shooter, intake, serializer);
    SuperStructure.setupElasticTab(shooter, intake);

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Emergency stop all superstructure subsystems on Left Bumper
    controller.leftBumper().onTrue(SuperStructure.stopAll(shooter, intake, serializer));

    // Run intake while holding Y button
    controller.y().whileTrue(SuperStructure.intakeStart(intake));

    // Snake Drive: Align heading with direction of motion when holding A button, with Intake
    // running
    controller
        .a()
        .whileTrue(
            Commands.parallel(
                DriveCommands.snakeDrive(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                SuperStructure.intakeStart(intake)));

    // Shoot for Hub (Shoot on the Move): Aim at velocity-compensated virtual Hub while driving on
    // Right Trigger
    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                DriveCommands.shootForHub(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                SuperStructure.shootForHub(drive, shooter, intake, serializer)));

    // Pass on the Move: Aim at pass target while driving on Left Trigger
    controller
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                DriveCommands.passOnTheMove(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                SuperStructure.pass(drive, shooter, intake, serializer)));

    // Zero gyro heading when Back button is pressed, and add dashboard controls
    controller.back().onTrue(DriveCommands.zeroGyro(drive));
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
