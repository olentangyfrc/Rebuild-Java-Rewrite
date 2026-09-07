package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
// import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem that manages Limelight vision cameras, feeds MegaTag1 and MegaTag2 vision
 * measurements to the drivetrain pose estimator, and logs vision data to AdvantageKit.
 */
public class Vision extends SubsystemBase {
  private final Drive drivetrain;
  private final List<String> cameraNames;

  private final Map<String, Pose2d> mt1Poses = new HashMap<>();
  // private final Map<String, Pose2d> mt2Poses = new HashMap<>();

  private final Map<String, List<PoseEstimate>> mt1MeasurementTracker = new HashMap<>();
  // private final Map<String, List<PoseEstimate>> mt2MeasurementTracker = new HashMap<>();

  private final Field2d visionField = new Field2d();
  private final Field2d LL1Field = new Field2d();

  // private final Field2d drivetrainField = new Field2d();
  // private final Field2d LL1Field = new Field2d();
  // private final Field2d LL2Field = new Field2d();
  // private final Field2d LL3Field = new Field2d();
  private final SendableChooser<String> disableChooser = new SendableChooser<>();

  private boolean sendYawRate = true;
  private double timeDelay = 0.5; // Seconds window to keep measurements
  private int maxFrameCount = 50;
  private int rewindCaptureCounter = 0;

  /**
   * Initializes Vision subsystem with a reference to the drivetrain and Limelight camera names.
   *
   * @param drivetrain
   * @param cameraNames
   */
  public Vision(Drive drivetrain, String... cameraNames) {
    this.drivetrain = drivetrain;

    if (cameraNames != null && cameraNames.length > 0) {
      this.cameraNames = Arrays.asList(cameraNames);
    } else {
      this.cameraNames = List.of("limelight-shooter", "limelight-left", "limelight-right");
    }

    for (String name : this.cameraNames) {
      mt1MeasurementTracker.put(name, new ArrayList<>());
      // mt2MeasurementTracker.put(name, new ArrayList
      mt1Poses.put(name, new Pose2d());
      // mt2Poses.put(name, new Pose2d());
    }

    visionField.setRobotPose(new Pose2d());
    LL1Field.setRobotPose(new Pose2d());
    setupLogging();
  }

  private void setupLogging() {
    disableChooser.setDefaultOption("None", "None");
    disableChooser.addOption("all", "all");

    for (String name : cameraNames) {
      disableChooser.addOption(name, name);
    }

    disableChooser.onChange(
        selected -> {
          if (selected != null && !"None".equals(selected)) {
            System.out.println("Limelight disabled state: " + selected);
          }
        });

    SmartDashboard.putData("Disable Chooser", disableChooser);
    SmartDashboard.putData("Vision Field", visionField);
    // SmartDashboard.putData("Drivetrain Field", drivetrainField);
    // SmartDashboard.putData("LL1 Field", LL1Field);
    // SmartDashboard.putData("LL2 Field", LL2Field);
    // SmartDashboard.putData("LL3 Field", LL3Field);
  }

  /** Configures IMU modes and initial orientation for all Limelights, then starts rewind. */
  public void setup() {
    double yawDegrees = drivetrain.getRotation().getDegrees();
    double yawRateDegreesPerSec =
        sendYawRate
            ? Units.radiansToDegrees(drivetrain.getChassisSpeeds().omegaRadiansPerSecond)
            : 0.0;

    for (String name : cameraNames) {
      LimelightHelpers.SetRobotOrientation(name, yawDegrees, yawRateDegreesPerSec, 0, 0, 0, 0);
      LimelightHelpers.SetIMUMode(name, 0); // 0 = use external gyro sent by robot
    }

    // startRewind();
  }

  public void slowDownProcessing() {
    for (String name : cameraNames) {
      LimelightHelpers.SetThrottle(name, 150);
    }
  }

  public void speedUpProcessing() {
    for (String name : cameraNames) {
      LimelightHelpers.SetThrottle(name, 0);
    }
  }

  /** Enables Limelight video rewind feature. */
  public void startRewind() {
    for (String name : List.of("limelight-left", "limelight-right")) {
      LimelightHelpers.setLimelightNTDouble(name, "rewind_enable_set", 1.0);
      LimelightHelpers.setLimelightNTDoubleArray(name, "capture_rewind", new double[] {0, 0});
    }
  }

  /** Captures rewind video for a specific duration. */
  public void captureRewind(double durationSeconds) {
    rewindCaptureCounter++;
    for (String name : List.of("limelight-left", "limelight-right")) {
      LimelightHelpers.setLimelightNTDoubleArray(
          name, "capture_rewind", new double[] {rewindCaptureCounter, durationSeconds});
    }
  }

  /** Captures rewind video with standard 165 second duration. */
  public void captureRewind() {
    captureRewind(165.0);
  }

  /** Enable or disable sending yaw rate for MegaTag2. */
  public void setSendYawRate(boolean sendYawRate) {
    this.sendYawRate = sendYawRate;
  }

  public void periodic() {
    double yawDegrees = drivetrain.getRotation().getDegrees();
    double yawRateDegreesPerSec =
        sendYawRate
            ? Units.radiansToDegrees(drivetrain.getChassisSpeeds().omegaRadiansPerSecond)
            : 0.0;

    String disabledSelection = disableChooser.getSelected();
    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    List<Pose2d> validPosesThisCycle = new ArrayList<>();

    for (String cameraName : cameraNames) {
      if ("all".equals(disabledSelection) || cameraName.equals(disabledSelection)) {
        continue;
      }

      // Update robot orientation in Limelight for MegaTag2 localization
      LimelightHelpers.SetRobotOrientation(
          cameraName, yawDegrees, yawRateDegreesPerSec, 0, 0, 0, 0);

      // // Fetch MegaTag2 pose estimate
      // PoseEstimate mt2Estimate =
      //     isRedAlliance
      //         ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraName)
      //         : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

      // Fetch MegaTag1 pose estimate
      PoseEstimate mt1Estimate =
          isRedAlliance
              ? LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName)
              : LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

      // Feed valid MT1 vision measurements to drivetrain pose estimator
      if (LimelightHelpers.validPoseEstimate(mt1Estimate)) {
        Matrix<N3, N1> stdDevs = calculateStdDevs(mt1Estimate);
        drivetrain.addVisionMeasurement(mt1Estimate.pose, mt1Estimate.timestampSeconds, stdDevs);

        mt1Poses.put(cameraName, mt1Estimate.pose);
        mt1MeasurementTracker.get(cameraName).add(mt1Estimate);
        validPosesThisCycle.add(mt1Estimate.pose);
      }

      // Track MT1 pose estimates
      if (LimelightHelpers.validPoseEstimate(mt1Estimate)) {
        mt1Poses.put(cameraName, mt1Estimate.pose);
        mt1MeasurementTracker.get(cameraName).add(mt1Estimate);
      }

      // Clean up stale measurements
      // cleanTracker(mt2MeasurementTracker.get(cameraName));
      cleanTracker(mt1MeasurementTracker.get(cameraName));
    }

    // Update vision field display and AdvantageKit outputs
    if (!validPosesThisCycle.isEmpty()) {
      Pose2d avgPose = getAveragePose(validPosesThisCycle);
      visionField.setRobotPose(avgPose);
    } else {
      visionField.setRobotPose(drivetrain.getPose());
    }
  }

  /** Calculates standard deviations dynamically based on AprilTag count and distance. */
  private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
    if (estimate.tagCount == 0) {
      return VecBuilder.fill(
          Units.inchesToMeters(36), Units.inchesToMeters(36), Units.degreesToRadians(30));
    }

    if (estimate.tagCount >= 2) {
      return VecBuilder.fill(0.3, 0.3, 0.3);
    } else {
      double distance = estimate.avgTagDist;
      double xyStdDev = 0.5 * Math.pow(distance, 2) / 2.0;
      double rotStdDev = 1.0 * Math.pow(distance, 2) / 2.0;
      return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }
  }

  /** Removes old measurements past timeDelay seconds or exceeding maxFrameCount. */
  private void cleanTracker(List<PoseEstimate> tracker) {
    double now = Timer.getFPGATimestamp();
    tracker.removeIf(m -> (now - m.timestampSeconds) > timeDelay);

    while (maxFrameCount > 0 && tracker.size() > maxFrameCount) {
      tracker.remove(0);
    }
  }

  /** Gets the most recent MegaTag1 pose estimate for the given Limelight. */
  public Pose2d getRecentMt1Pose(String limelightName) {
    return mt1Poses.getOrDefault(limelightName, new Pose2d());
  }

  /** Gets the most recent MegaTag2 pose estimate for the given Limelight. */
  // public Pose2d getRecentMt2Pose(String limelightName) {
  //   return mt2Poses.getOrDefault(limelightName, new Pose2d());
  // }

  /** Gets estimated FPS for a given Limelight camera over the tracking time window. */
  public double getRecentFps(String limelightName) {
    List<PoseEstimate> tracker = mt1MeasurementTracker.get(limelightName);
    if (tracker == null || timeDelay <= 0) {
      return 0.0;
    }
    return tracker.size() / timeDelay;
  }

  /**
   * Checks if any Limelight camera has received fresh vision measurements within staleCountTime.
   */
  public boolean hasVision(double staleCountTime) {
    double now = Timer.getFPGATimestamp();
    for (List<PoseEstimate> tracker : mt1MeasurementTracker.values()) {
      if (!tracker.isEmpty()) {
        PoseEstimate last = tracker.get(tracker.size() - 1);
        if (now - last.timestampSeconds < staleCountTime) {
          return true;
        }
      }
    }
    return false;
  }

  /** Calculates the average pose from a list of Pose2d objects. */
  public static Pose2d getAveragePose(List<Pose2d> poses) {
    if (poses == null || poses.isEmpty()) {
      return new Pose2d();
    }
    if (poses.size() == 1) {
      return poses.get(0);
    }

    double xSum = 0.0;
    double ySum = 0.0;
    double sinSum = 0.0;
    double cosSum = 0.0;

    for (Pose2d pose : poses) {
      xSum += pose.getX();
      ySum += pose.getY();
      sinSum += pose.getRotation().getSin();
      cosSum += pose.getRotation().getCos();
    }

    int count = poses.size();
    double avgX = xSum / count;
    double avgY = ySum / count;
    double avgRot = Math.atan2(sinSum / count, cosSum / count);

    return new Pose2d(avgX, avgY, new Rotation2d(avgRot));
  }

  /** Varargs overload for getAveragePose. */
  public static Pose2d getAveragePose(Pose2d... poses) {
    return getAveragePose(Arrays.asList(poses));
  }
}
