package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double[] botposeWpiBlue = new double[0];
    public double[] botposeOrbWpiBlue = new double[0];
    public boolean hasTarget = false;
    public double timestamp = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
