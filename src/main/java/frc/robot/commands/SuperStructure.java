package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SuperStructure {
  private final Drive drive;
  private final Shooter shooter;

  private static final LoggedDashboardNumber distanceOverride =
      new LoggedDashboardNumber("Shooter/ManualDistanceOverrideMeters", 0.0);

  public SuperStructure(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
  }

  /**
   * Command to shoot for the hub using drivetrain pose or manual distance override.
   */
  public Command shoot() {
    return Commands.run(
        () -> {
          double override = distanceOverride.get();
          double distance = (override > 0.0) ? override : drive.getDistanceFromHub();
          shooter.shootForHub(distance);
        },
        shooter);
  }
}


