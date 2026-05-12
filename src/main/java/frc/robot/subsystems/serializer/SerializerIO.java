package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.AutoLog;

public interface SerializerIO {
  @AutoLog
  public static class SerializerIOInputs {
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
    public String state = "IDLE";
  }

  public default void updateInputs(SerializerIOInputs inputs) {}

  public default void runVelocity(double velocityRps) {}

  public default void setRollerVelocity(double velocityRps) {}

  public default void stop() {}
}
