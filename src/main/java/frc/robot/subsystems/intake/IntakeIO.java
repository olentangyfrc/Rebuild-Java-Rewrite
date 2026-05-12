package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotAngleRads = 0.0;
    public double pivotVelocityRadsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;

    public double rollerVelocityRps = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public String state = "IDLE";
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotVoltage(double volts) {}

  public default void setRollerVelocity(double velocityRps) {}

  public default void stop() {}
}
