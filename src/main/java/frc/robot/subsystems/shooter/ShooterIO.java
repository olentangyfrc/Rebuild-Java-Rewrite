package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftBottomVelocityRps = 0.0;
    public double leftTopVelocityRps = 0.0;
    public double rightBottomVelocityRps = 0.0;
    public double rightTopVelocityRps = 0.0;

    public double indexer1VelocityRps = 0.0;
    public double indexer2VelocityRps = 0.0;

    public double hoodAngleRads = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVelocity(double velocityRps) {}

  public default void setIndexerVelocity(double indexer1Rps, double indexer2Rps) {}

  public default void setHoodVoltage(double volts) {}

  public default void stop() {}
}
