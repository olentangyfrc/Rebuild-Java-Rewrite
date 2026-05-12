package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;

public class SuperstructureCommands {
  private final Intake intake;
  private final Shooter shooter;
  private final Serializer serializer;

  public SuperstructureCommands(Intake intake, Shooter shooter, Serializer serializer) {
    this.intake = intake;
    this.shooter = shooter;
    this.serializer = serializer;
  }

  public Command stashCommand() {
    return Commands.parallel(intake.stashCommand(), serializer.stopCommand(), shooter.idleCommand())
        .withName("SuperstructureStash");
  }

  public Command idleCommand() {
    return Commands.parallel(
            intake.deployCommand(), serializer.stopCommand(), shooter.idleCommand())
        .withName("SuperstructureIdle");
  }

  public Command intakeFuelCommand() {
    return Commands.parallel(
            intake.intakeCommand(), serializer.stopCommand(), shooter.idleCommand())
        .withName("SuperstructureIntake");
  }

  public Command shootFuelCommand(double distanceMeters) {
    // Simple mock interpolation for now
    double angle = Math.toRadians(35);
    double rpm = 2500;

    return Commands.sequence(
            Commands.runOnce(() -> shooter.setShootParams(rpm, angle)),
            Commands.parallel(
                    shooter.spinUpCommand(), intake.deployCommand(), serializer.runSlowReverse())
                .until(shooter::atTargetSpeed),
            Commands.parallel(shooter.shootCommand(), serializer.runForward()))
        .withName("SuperstructureShoot");
  }

  public Command unjamCommand() {
    return Commands.parallel(serializer.runReverse(), shooter.ejectCommand())
        .withName("SuperstructureUnjam");
  }

  public Command ejectFuelCommand() {
    return Commands.parallel(serializer.runReverse(), intake.ejectCommand())
        .withName("SuperstructureEject");
  }
}
