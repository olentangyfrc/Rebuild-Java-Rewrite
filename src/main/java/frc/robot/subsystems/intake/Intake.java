package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum IntakeState {
    IDLE,
    DEPLOYED,
    INTAKE,
    INTAKE_PASS,
    AGITATE,
    EJECT,
    STASHED,
    EMERGENCY_STASH
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final ProfiledPIDController pivotPid;
  private final ArmFeedforward pivotFf;

  private static final double MAX_PIVOT_POSITION = Math.toRadians(116);
  private static final double MIN_PIVOT_POSITION = Math.toRadians(0.5);

  private IntakeState state = IntakeState.DEPLOYED;
  private double targetPivotAngle = Math.toRadians(-1);
  private double targetRollerVelocity = 0.0;

  private final Timer agitationTimer = new Timer();

  public Intake(IntakeIO io) {
    this.io = io;
    pivotFf = new ArmFeedforward(0.0, 0.42, 0.17, 0.01);
    pivotPid =
        new ProfiledPIDController(
            8.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 4));
    pivotPid.setTolerance(Math.toRadians(5));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    handleStateLogic();
    Logger.processInputs("Intake", inputs);

    double clampedAngle = MathUtil.clamp(targetPivotAngle, MIN_PIVOT_POSITION, MAX_PIVOT_POSITION);
    double pivotVolts =
        pivotPid.calculate(inputs.pivotAngleRads, clampedAngle)
            + pivotFf.calculate(clampedAngle, 0);

    io.setPivotVoltage(pivotVolts);

    // Roller logic from python: stop if angle > 42 deg
    if (targetRollerVelocity == 0 || (inputs.pivotAngleRads > Math.toRadians(42))) {
      io.setRollerVelocity(0);
    } else {
      io.setRollerVelocity(targetRollerVelocity);
    }

    inputs.state = state.name();
  }

  private void handleStateLogic() {
    switch (state) {
      case IDLE:
        targetRollerVelocity = 0;
        targetPivotAngle = MAX_PIVOT_POSITION - Math.toRadians(1);
        break;

      case DEPLOYED:
        targetPivotAngle = Math.toRadians(-1);
        targetRollerVelocity = -5; // Keep fuel from leaking out
        break;

      case INTAKE:
        targetRollerVelocity = -90;
        targetPivotAngle = Math.toRadians(-1);
        break;

      case INTAKE_PASS:
        if (inputs.pivotAngleRads < Math.toRadians(30)) {
          targetRollerVelocity = -40;
        }
        targetPivotAngle = 0;
        break;

      case AGITATE:
        if (agitationTimer.get() == 0) agitationTimer.start();

        if (agitationTimer.get() > 1.5) {
          state = IntakeState.STASHED;
          agitationTimer.stop();
          agitationTimer.reset();
        } else if (agitationTimer.get() > 1.0) {
          targetPivotAngle = Math.toRadians(60);
        } else if (agitationTimer.get() > 0.65) {
          targetPivotAngle = Math.toRadians(25);
        }
        targetRollerVelocity = -15;
        break;

      case EJECT:
        targetRollerVelocity = 90;
        targetPivotAngle = 0;
        break;

      case STASHED:
        targetRollerVelocity = 0;
        targetPivotAngle = MAX_PIVOT_POSITION - Math.toRadians(1);
        break;

      case EMERGENCY_STASH:
        targetRollerVelocity = -30;
        targetPivotAngle = MAX_PIVOT_POSITION - Math.toRadians(1);
        break;
    }
  }

  public void setState(IntakeState newState) {
    this.state = newState;
  }

  public IntakeState getState() {
    return this.state;
  }

  public Command idleCommand() {
    return runOnce(() -> state = IntakeState.IDLE).withName("IntakeIdle");
  }

  public Command deployCommand() {
    return runOnce(() -> state = IntakeState.DEPLOYED).withName("IntakeDeploy");
  }

  public Command intakeCommand() {
    return runOnce(() -> state = IntakeState.INTAKE).withName("IntakeIntake");
  }

  public Command stashCommand() {
    return runOnce(() -> state = IntakeState.STASHED).withName("IntakeStash");
  }

  public Command agitateCommand() {
    return runOnce(
            () -> {
              state = IntakeState.AGITATE;
              agitationTimer.restart();
            })
        .withName("IntakeAgitate");
  }

  public Command ejectCommand() {
    return runOnce(() -> state = IntakeState.EJECT).withName("IntakeEject");
  }

  public boolean atTargetPosition() {
    return pivotPid.atGoal();
  }
}
