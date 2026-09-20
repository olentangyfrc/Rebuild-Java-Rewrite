package frc.robot.commands;

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

  /** Command to shoot for the hub using drivetrain pose or manual distance override. */
  public static Command shoot(Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
        () -> {
          boolean usePose = useDrivetrainPose.get();
          double distance = usePose ? drive.getDistanceFromHub() : distanceOverride.get();
          shooter.shootForHub(distance);
          if (shooter.isDrumAtSpeed() && shooter.isHoodAtSetpoint()) {

            shooter.startfeed();
            serializer.start();

            intake.startagitationIntake();
            // intake.setPivotSetPoint(0);

          } else {
            shooter.waitforfeed();
            intake.resetIntake();
            serializer.stop();
          }
        },
        shooter,
        intake,
        serializer);
  }

  /** Command to shoot on the move for the hub using velocity-compensated distance. */
  public static Command shootOnTheMove(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
        () -> {
          boolean usePose = useDrivetrainPose.get();
          double distance = usePose ? drive.getShootOnTheMoveDistance() : distanceOverride.get();
          shooter.shootForHub(distance);
          if (shooter.isDrumAtSpeed() && shooter.isHoodAtSetpoint()) {

            shooter.startfeed();
            serializer.start();

            intake.startagitationIntake();

          } else {
            shooter.waitforfeed();
            intake.resetIntake();
            serializer.stop();
          }
        },
        shooter,
        intake,
        serializer);
  }

  public static Command stopAll(Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
        () -> {
          shooter.stop();
          shooter.resetHood();
          shooter.stopfeed();
          intake.resetIntake();
          serializer.stop();
        },
        shooter,
        intake,
        serializer);
  }

  public Command shoot() {
    return shoot(drive, shooter, intake, serializer);
  }

  public static Command intakeSTART(Intake intake) {
    return Commands.runEnd(
        () -> {
          intake.start();
          intake.setPivotSetPoint(0);
        },
        () -> intake.stop(),
        intake);
  }

  public static Command intakeWithDrive(Intake intake, double downwardVoltage) {
    return Commands.runEnd(
        () -> intake.driveIntakeDown(downwardVoltage), () -> intake.resetIntake(), intake);
  }

  // HELPER MEATHOD DONT GET RID OF ME
  public Command intakeWithDrive(double downwardVoltage) {
    return intakeWithDrive(intake, downwardVoltage);
  }
}
