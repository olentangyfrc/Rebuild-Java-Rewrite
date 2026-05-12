package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] ios;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Drive drive;

  public Vision(Drive drive, VisionIO... ios) {
    this.ios = ios;
    this.drive = drive;
    this.inputs = new VisionIOInputsAutoLogged[ios.length];
    for (int i = 0; i < ios.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);

      // Add standard mt1 pose
      if (inputs[i].hasTarget && inputs[i].botposeWpiBlue.length >= 7) {
        double x = inputs[i].botposeWpiBlue[0];
        double y = inputs[i].botposeWpiBlue[1];
        double yaw = inputs[i].botposeWpiBlue[5];

        // Discard empty or 0,0 poses
        if (Math.abs(x) > 0.01 && Math.abs(y) > 0.01) {
          Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));

          // X/Y stddev is approx 0.1, Rot is 0.1 for standard
          drive.addVisionMeasurement(pose, inputs[i].timestamp, VecBuilder.fill(0.1, 0.1, 0.1));
        }
      }

      // Add ORB mt2 pose
      if (inputs[i].hasTarget && inputs[i].botposeOrbWpiBlue.length >= 7) {
        double x = inputs[i].botposeOrbWpiBlue[0];
        double y = inputs[i].botposeOrbWpiBlue[1];
        double yaw = inputs[i].botposeOrbWpiBlue[5];

        if (Math.abs(x) > 0.01 && Math.abs(y) > 0.01) {
          Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));

          // ORB might be more/less accurate, using standard stddev for now
          drive.addVisionMeasurement(pose, inputs[i].timestamp, VecBuilder.fill(0.1, 0.1, 0.1));
        }
      }
    }
  }
}
