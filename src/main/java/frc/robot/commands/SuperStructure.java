package frc.robot.commands;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure {
  private final Drive drive;
  private final Shooter shooter;

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

  public SuperStructure(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
  }

  /** Command to shoot for the hub using drivetrain pose or manual distance override. */
  public static Command shoot(Drive drive, Shooter shooter) {
    return Commands.run(
        () -> {
          boolean usePose = useDrivetrainPose.get();
          double distance = usePose ? drive.getDistanceFromHub() : distanceOverride.get();
          shooter.shootForHub(distance);
        },
        shooter);
  }

  public static Command stopAll(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stop();
          shooter.resetHood();
        },
        shooter);
  }

  public Command shoot() {
    return shoot(drive, shooter);
  }
}
