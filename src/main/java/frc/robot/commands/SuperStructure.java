package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShiftScheduler;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure {
  private static String lastCommand = "None";

  public static String getLastCommand() {
    return lastCommand;
  }

  private static void setLastCommand(String commandName) {
    lastCommand = commandName;
    Logger.recordOutput("SuperStructure/LastCommand", commandName);
    SmartDashboard.putString("SuperStructure/LastCommand", commandName);
  }

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
    setLastCommand("None");
  }

  public SuperStructure(Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.serializer = serializer;
  }

  /**
   * Sets up the Elastic Dashboard / Shuffleboard tab displaying last called SuperStructure command.
   */
  public static void setupElasticTab(Shooter shooter, Intake intake) {
    ShuffleboardTab tab = Shuffleboard.getTab("SuperStructure");

    tab.addString("Last Command", () -> lastCommand).withPosition(0, 0).withSize(3, 2);

    tab.addBoolean("Shooter Drum Ready", shooter::isDrumAtSpeed).withPosition(3, 0).withSize(2, 1);

    tab.addBoolean("Shooter Hood Ready", shooter::isHoodAtSetpoint)
        .withPosition(3, 1)
        .withSize(2, 1);

    tab.addBoolean("Passing Active", shooter::isPassing).withPosition(5, 0).withSize(2, 1);
  }

  /**
   * Command to shoot for the hub using velocity-compensated distance when drum, hood, AND
   * drivetrain heading are aligned.
   */
  public static Command shootForHub(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
            () -> {
              boolean usePose = useDrivetrainPose.get();
              double distance = usePose ? drive.getShootForHubDistance() : distanceOverride.get();
              shooter.shootForHub(distance);
              if (shooter.isDrumAtSpeed() && shooter.isHoodAtSetpoint() && drive.isAlignedToHub()) {
                shooter.startFeed();
                serializer.start();
                intake.startAgitationIntake();
                ShiftScheduler.setFeedingActive(true);
              } else {
                shooter.waitForFeed();
                intake.resetIntake();
                serializer.stop();
                ShiftScheduler.setFeedingActive(false);
              }
            },
            shooter,
            intake,
            serializer)
        .beforeStarting(() -> setLastCommand("shootForHub"))
        .finallyDo(
            () -> {
              shooter.stop();
              shooter.resetHood();
              shooter.stopFeed();
              serializer.stop();
              intake.resetIntake();
              ShiftScheduler.setFeedingActive(false);
            });
  }

  public static Command pass(Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
            () -> {
              boolean usePose = useDrivetrainPose.get();
              double distance = usePose ? drive.getDistanceFromPass() : distanceOverride.get();
              shooter.pass(distance);
              if (shooter.isDrumAtSpeed()
                  && shooter.isHoodAtSetpoint()
                  && drive.isAlignedToPass()) {
                shooter.startfeed();
                serializer.start();
                intake.startagitationIntake();
                ShiftScheduler.setFeedingActive(true);
              } else {
                shooter.waitforfeed();
                intake.resetIntake();
                serializer.stop();
                ShiftScheduler.setFeedingActive(false);
              }
            },
            shooter,
            intake,
            serializer)
        .beforeStarting(() -> setLastCommand("pass"))
        .finallyDo(
            () -> {
              shooter.stop();
              shooter.resetHood();
              shooter.stopFeed();
              serializer.stop();
              intake.resetIntake();
              ShiftScheduler.setFeedingActive(false);
            });
  }

  public static Command shootOnTheMove(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    return shootForHub(drive, shooter, intake, serializer);
  }

  public static Command stopAll(Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.runOnce(
        () -> {
          setLastCommand("stopAll");
          shooter.stop();
          shooter.resetHood();
          shooter.stopFeed();
          intake.resetIntake();
          serializer.stop();
          ShiftScheduler.setFeedingActive(false);
        },
        shooter,
        intake,
        serializer);
  }

  public static Command intakeStart(Intake intake) {
    return Commands.run(
            () -> {
              intake.start();
              intake.setPivotSetPoint(0);
            },
            intake)
        .beforeStarting(() -> setLastCommand("intakeStart"))
        .finallyDo(intake::resetIntake);
  }

  public static Command intakeStartAuto(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.start();
              intake.setPivotSetPoint(0);
            },
            intake)
        .beforeStarting(() -> setLastCommand("intakeStartAuto"));
  }

  public static Command intakeSTARTAuto(Intake intake) {
    return intakeStartAuto(intake);
  }

  public static Command intakeSTART(Intake intake) {
    return intakeStart(intake);
  }

  public static Command stashIntake(Intake intake) {
    return Commands.run(() -> intake.stash(), intake)
        .beforeStarting(() -> setLastCommand("stashIntake"))
        .finallyDo(intake::resetIntake);
  }

  public static Command ejectFuel(Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
            () -> {
              intake.eject();
              serializer.reverse();
              shooter.reverseFeed();
            },
            shooter,
            intake,
            serializer)
        .beforeStarting(() -> setLastCommand("ejectFuel"))
        .finallyDo(
            () -> {
              intake.stop();
              serializer.stop();
              shooter.stopFeed();
            });
  }

  public static Command unjam(Shooter shooter, Intake intake, Serializer serializer) {
    return Commands.run(
            () -> {
              intake.eject();
              serializer.reverse();
              shooter.unjam();
            },
            shooter,
            intake,
            serializer)
        .beforeStarting(() -> setLastCommand("unjam"))
        .finallyDo(
            () -> {
              intake.stop();
              serializer.stop();
              shooter.stop();
            });
  }

  public static Command smartShoot(
      Drive drive,
      Shooter shooter,
      Intake intake,
      Serializer serializer,
      Supplier<Boolean> auxStashSupplier,
      Supplier<Boolean> auxIntakeSupplier) {
    return Commands.run(
            () -> {
              boolean isHubShot = drive.getPose().getX() < 5.2;

              if (isHubShot) {
                boolean usePose = useDrivetrainPose.get();
                double distance = usePose ? drive.getShootForHubDistance() : distanceOverride.get();
                shooter.shootForHub(distance);
              } else {
                boolean usePose = useDrivetrainPose.get();
                double distance = usePose ? drive.getDistanceFromPass() : distanceOverride.get();
                shooter.pass(distance);
              }

              boolean aligned = isHubShot ? drive.isAlignedToHub() : drive.isAlignedToPass();

              if (shooter.isDrumAtSpeed() && shooter.isHoodAtSetpoint() && aligned) {
                shooter.startFeed();
                serializer.start();

                if (auxStashSupplier.get()) {
                  intake.stash();
                } else if (auxIntakeSupplier.get()) {
                  intake.start();
                  intake.setPivotSetPoint(0);
                } else {
                  intake.startAgitationIntake();
                }
                ShiftScheduler.setFeedingActive(true);
              } else {
                shooter.waitForFeed();
                if (auxStashSupplier.get()) {
                  intake.stash();
                } else {
                  intake.resetIntake();
                }
                serializer.stop();
                ShiftScheduler.setFeedingActive(false);
              }
            },
            shooter,
            intake,
            serializer)
        .beforeStarting(() -> setLastCommand("smartShoot"))
        .finallyDo(
            () -> {
              shooter.stop();
              shooter.resetHood();
              shooter.stopFeed();
              serializer.stop();
              intake.resetIntake();
              ShiftScheduler.setFeedingActive(false);
            });
  }

  // Instance command helpers
  public Command shootForHub() {
    return shootForHub(drive, shooter, intake, serializer);
  }

  public Command shootOnTheMove() {
    return shootForHub();
  }

  public Command stopAll() {
    return stopAll(shooter, intake, serializer);
  }

  public Command intakeStart() {
    return intakeStart(intake);
  }

  public Command intakeSTART() {
    return intakeStart();
  }

  public Command intake() {
    return intakeStart(intake);
  }

  /**
   * Command to drive along the X axis until targetX is reached while simultaneously intaking fuel
   * and warming up the shooter to 1800 RPM.
   */
  public static Command bump(
      Drive drive,
      Shooter shooter,
      Intake intake,
      double targetX,
      double bumpSpeed,
      double targetY) {
    return DriveCommands.bump(drive, targetX, bumpSpeed, targetY)
        .deadlineWith(
            intakeStart(intake), Commands.run(() -> shooter.setDrumVelocity(1800.0), shooter))
        .beforeStarting(() -> setLastCommand("bump"));
  }

  public static Command bump(
      Drive drive, Shooter shooter, Intake intake, double targetX, double bumpSpeed) {
    return DriveCommands.bump(drive, targetX, bumpSpeed)
        .deadlineWith(
            intakeStart(intake), Commands.run(() -> shooter.setDrumVelocity(1800.0), shooter))
        .beforeStarting(() -> setLastCommand("bump"));
  }

  public Command bump(double targetX, double bumpSpeed) {
    return bump(drive, shooter, intake, targetX, bumpSpeed);
  }

  public Command bump(double targetX, double bumpSpeed, double targetY) {
    return bump(drive, shooter, intake, targetX, bumpSpeed, targetY);
  }

  /** Command to line up the drivetrain to a target pose while running intake. */
  public static Command lineUp(Drive drive, Intake intake, Pose2d targetPose, boolean yOnly) {
    return DriveCommands.lineUp(drive, targetPose, yOnly)
        .deadlineWith(intakeStart(intake))
        .beforeStarting(() -> setLastCommand("lineUp"));
  }

  public static Command lineUp(Drive drive, Intake intake, Pose2d targetPose) {
    return lineUp(drive, intake, targetPose, true);
  }

  public static Command botLineUpLeft(Drive drive, Intake intake) {
    return lineUp(drive, intake, new Pose2d(2.5, 7.51, Rotation2d.kZero), true)
        .beforeStarting(() -> setLastCommand("botLineUpLeft"));
  }

  public static Command botLineUpRight(Drive drive, Intake intake) {
    return lineUp(drive, intake, new Pose2d(2.95, 0.50, Rotation2d.kZero), true)
        .beforeStarting(() -> setLastCommand("botLineUpRight"));
  }

  public Command botLineUpLeft() {
    return botLineUpLeft(drive, intake);
  }

  public Command botLineUpRight() {
    return botLineUpRight(drive, intake);
  }

  /** Registers PathPlanner named commands for SuperStructure actions. */
  public static void registerNamedCommands(
      Drive drive, Shooter shooter, Intake intake, Serializer serializer) {
    NamedCommands.registerCommand(
        "shootForHub",
        Commands.parallel(
                DriveCommands.shootForHub(drive), shootForHub(drive, shooter, intake, serializer))
            .withTimeout(4.0));

    NamedCommands.registerCommand("intake", intakeStartAuto(intake));
    NamedCommands.registerCommand("stopAll", stopAll(shooter, intake, serializer));
    NamedCommands.registerCommand("bump", bump(drive, shooter, intake, 3.0, -3.3));
    NamedCommands.registerCommand("botlineupleft", botLineUpLeft(drive, intake));
    NamedCommands.registerCommand("botlineupright", botLineUpRight(drive, intake));
  }

  /** Registers a custom bump NamedCommand with specified targetX and speed. */
  public static void registerBumpCommand(
      Drive drive,
      Shooter shooter,
      Intake intake,
      String commandName,
      double targetX,
      double bumpSpeed) {
    NamedCommands.registerCommand(commandName, bump(drive, shooter, intake, targetX, bumpSpeed));
  }

  /** Registers PathPlanner named commands using this SuperStructure instance. */
  public void registerNamedCommands() {
    registerNamedCommands(drive, shooter, intake, serializer);
  }
}
