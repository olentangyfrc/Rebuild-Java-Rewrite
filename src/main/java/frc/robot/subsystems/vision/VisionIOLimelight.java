package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {
  private final DoubleArraySubscriber botposeWpiBlueSub;
  private final DoubleArraySubscriber botposeOrbWpiBlueSub;
  private final DoubleSubscriber tvSub;
  private final DoubleSubscriber clSub;
  private final DoubleSubscriber tlSub;

  public VisionIOLimelight(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    botposeWpiBlueSub = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    botposeOrbWpiBlueSub =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    tvSub = table.getDoubleTopic("tv").subscribe(0.0);
    clSub = table.getDoubleTopic("cl").subscribe(0.0);
    tlSub = table.getDoubleTopic("tl").subscribe(0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.botposeWpiBlue = botposeWpiBlueSub.get();
    inputs.botposeOrbWpiBlue = botposeOrbWpiBlueSub.get();
    inputs.hasTarget = tvSub.get() == 1.0;

    // Limelight latency = capture latency + pipeline latency
    double latencyMs = clSub.get() + tlSub.get();
    inputs.timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
  }
}
