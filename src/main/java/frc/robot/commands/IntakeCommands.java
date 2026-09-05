package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

  Intake intake;

  public static Command setPivotAngle(Intake intake, double pivotAngle) {
    return Commands.run(
        () -> {
          intake.setPivotSetPoint(pivotAngle);
        },
        intake);
  }

  public static Command startIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.start();
        },
        intake);
  }

  public static Command stopIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.stop();
        },
        intake);
  }

  public static Command ejectIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.eject();
        },
        intake);
  }

  public static Command idleIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.idle();
        },
        intake);
  }
}
