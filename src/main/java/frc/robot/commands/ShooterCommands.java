package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

  Shooter shooter;

  public static Command spinUp(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.spinUp();
        },
        shooter);
  }

  public static Command stop(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stop();
        },
        shooter);
  }

  public static Command setHoodAngle(Shooter shooter, double hoodAngle) {
    return Commands.run(
        () -> {
          shooter.setHoodSetPoint(hoodAngle);
        },
        shooter);
  }
}
