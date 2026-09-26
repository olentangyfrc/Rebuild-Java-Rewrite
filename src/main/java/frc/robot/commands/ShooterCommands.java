package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  private ShooterCommands() {}

  public static Command setDrumVelocity(Shooter shooter, double velocity) {
    return Commands.run(() -> shooter.setDrumVelocity(velocity), shooter);
  }

  @Deprecated
  public static Command SetDrumVelocity(Shooter shooter, double velocity) {
    return setDrumVelocity(shooter, velocity);
  }

  public static Command spinUpDrum(Shooter shooter) {
    return Commands.run(() -> shooter.spinUpDrum(), shooter)
        .finallyDo(
            () -> {
              shooter.stop();
              shooter.resetHood();
            });
  }

  public static Command stop(Shooter shooter) {
    return Commands.run(() -> shooter.stop(), shooter);
  }

  public static Command setHoodAngle(Shooter shooter, double hoodAngle) {
    return Commands.run(() -> shooter.setHoodSetPoint(hoodAngle), shooter);
  }

  public static Command shootForHub(Shooter shooter, double distanceMeters) {
    return Commands.run(() -> shooter.shootForHub(distanceMeters), shooter);
  }
}
