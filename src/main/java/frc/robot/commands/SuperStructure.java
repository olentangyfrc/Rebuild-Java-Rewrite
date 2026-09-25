package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Serializer serializer;

  // Native WPILib entries for live NetworkTables tuning/toggling
  private static final BooleanEntry useDrivetrainPose =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getBooleanTopic("Shooter/UseDrivetrainPose")
          .getEntry(true);

  private static final DoubleEntry distanceOverride =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleTopic("Shooter/ManualDistanceOverrideMeters")
          .getEntry(0.0);

  static {
    useDrivetrainPose.setDefault(true);
    distanceOverride.setDefault(0.0);
  }

  public SuperStructure(Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.serializer = serializer;
  }

  /** Command to shoot for the hub using velocity-compensated distance when moving. */
  public static Command shootForHub(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
        () -> {
          boolean usePose = useDrivetrainPose.get();
          double distance = usePose ? drive.getShootForHubDistance() : distanceOverride.get();
          shooter.shootForHub(distance);
          if (shooter.isDrumAtSpeed() && shooter.isHoodAtSetpoint()) {
            shooter.startFeed();
            serializer.start();
            intake.startAgitationIntake();
          } else {
            shooter.waitForFeed();
            intake.resetIntake();
            serializer.stop();
          }
        },
        shooter,
        intake,
        serializer);
  }

  /** Deprecated alias for {@link #shootForHub(Drive, Shooter, Intake, Serializer)}. */
  @Deprecated
  public static Command shootOnTheMove(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return shootForHub(drive, shooter, intake, serializer);
  }

  public static Command stopAll(Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.runOnce(
        () -> {
          shooter.stop();
          shooter.resetHood();
          shooter.stopFeed();
          intake.resetIntake();
          serializer.stop();
        },
        shooter,
        intake,
        serializer);
  }

  public static Command intakeStart(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.start();
          intake.setPivotSetPoint(0);
        },
        intake);
  }

  @Deprecated
  public static Command intakeSTART(Intake intake) {
    return intakeStart(intake);
  }

  // Instance command helpers
  public Command shootForHub() {
    return shootForHub(drive, shooter, intake, serializer);
  }

  @Deprecated
  public Command shootOnTheMove() {
    return shootForHub();
  }

  public Command stopAll() {
    return stopAll(shooter, intake, serializer);
  }

  public Command intakeStart() {
    return intakeStart(intake);
  }

  @Deprecated
  public Command intakeSTART() {
    return intakeStart();
  }

  public Command intake() {
    return intakeStart(intake);
  }

  /** Registers PathPlanner named commands for SuperStructure actions. */
  public static void registerNamedCommands(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    NamedCommands.registerCommand(
        "shootForHub", shootForHub(drive, shooter, intake, serializer).withTimeout(2.0));
    NamedCommands.registerCommand(
        "shootOnTheMove", shootForHub(drive, shooter, intake, serializer).withTimeout(2.0));
    NamedCommands.registerCommand("intake", intakeStart(intake));
    NamedCommands.registerCommand("stopAll", stopAll(shooter, intake, serializer));
  }

  /** Registers PathPlanner named commands using this SuperStructure instance. */
  public void registerNamedCommands() {
    registerNamedCommands(drive, shooter, intake, serializer);
  }
}
