package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final ProfiledPIDController pivotPid;
  private final ArmFeedforward pivotFf;

  private static final double MAX_PIVOT_POSITION = Math.toRadians(116);
  private static final double MIN_PIVOT_POSITION = Math.toRadians(0.5);

  private double targetPivotAngle = MAX_PIVOT_POSITION - Math.toRadians(1);
  private double targetRollerRps = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;

    pivotFf = new ArmFeedforward(0.1, 0.35, 0.0, 0.0);
    pivotPid = new ProfiledPIDController(4.0, 0.0, 0.1, new TrapezoidProfile.Constraints(500, 250));
    pivotPid.setTolerance(Math.toRadians(5));
    pivotPid.reset(inputs.pivotAngleRads);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    double clampedTarget = MathUtil.clamp(targetPivotAngle, MIN_PIVOT_POSITION, MAX_PIVOT_POSITION);
    pivotPid.setGoal(clampedTarget);

    double pivotVoltage =
        pivotPid.calculate(inputs.pivotAngleRads) + pivotFf.calculate(inputs.pivotAngleRads, 0);
    io.setPivotVoltage(pivotVoltage);

    if (targetRollerRps == 0 || inputs.pivotAngleRads > Math.toRadians(42)) {
      io.setRollerVelocity(0);
    } else {
      io.setRollerVelocity(targetRollerRps);
    }
  }

  public void setGoal(double angleRads, double rollerRps) {
    this.targetPivotAngle = angleRads;
    this.targetRollerRps = rollerRps;
  }

  public Command idleCommand() {
    return run(() -> setGoal(MAX_PIVOT_POSITION - Math.toRadians(1), 0.0)).withName("IntakeIdle");
  }

  public Command deployCommand() {
    return run(() -> setGoal(Math.toRadians(-1), -5.0)).withName("IntakeDeploy");
  }

  public Command intakeCommand() {
    return run(() -> setGoal(Math.toRadians(-1), -90.0)).withName("IntakeIn");
  }

  public Command intakePassCommand() {
    return run(() -> setGoal(0.0, inputs.pivotAngleRads < Math.toRadians(30) ? -40.0 : 0.0))
        .withName("IntakePass");
  }

  public Command ejectCommand() {
    return run(() -> setGoal(0.0, 90.0)).withName("IntakeEject");
  }

  public Command stashCommand() {
    return run(() -> setGoal(MAX_PIVOT_POSITION - Math.toRadians(1), 0.0)).withName("IntakeStash");
  }

  public Command emergencyStashCommand() {
    return run(() -> setGoal(MAX_PIVOT_POSITION - Math.toRadians(1), -30.0))
        .withName("IntakeEmergencyStash");
  }
}
