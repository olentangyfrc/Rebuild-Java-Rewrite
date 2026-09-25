package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command setPivotAngle(Intake intake, double pivotAngle) {
    return Commands.run(() -> intake.setPivotSetPoint(pivotAngle), intake);
  }

  public static Command driveIntakeDown(Intake intake, double voltage) {
    return Commands.runEnd(
        () -> intake.driveIntakeDown(voltage), () -> intake.resetIntake(), intake);
  }

  public static Command startIntake(Intake intake) {
    return Commands.run(() -> intake.start(), intake);
  }

  public static Command startAgitationIntake(Intake intake) {
    return Commands.run(() -> intake.startAgitationIntake(), intake);
  }

  @Deprecated
  public static Command startagitationIntake(Intake intake) {
    return startAgitationIntake(intake);
  }

  public static Command stopAgitationIntake(Intake intake) {
    return Commands.run(() -> intake.stopAgitationIntake(), intake);
  }

  @Deprecated
  public static Command stopagitationIntake(Intake intake) {
    return stopAgitationIntake(intake);
  }

  public static Command stopIntake(Intake intake) {
    return Commands.run(() -> intake.stop(), intake);
  }

  public static Command ejectIntake(Intake intake) {
    return Commands.run(() -> intake.eject(), intake);
  }

  public static Command idleIntake(Intake intake) {
    return Commands.run(() -> intake.intakeIdle(), intake);
  }
}
