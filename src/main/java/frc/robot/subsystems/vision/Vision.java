package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
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

/**
 * Vision subsystem that manages Limelight cameras, processes MegaTag1 measurements in WPILib Blue
 * Alliance coordinates (wpiBlue), feeds pose measurements to the drivetrain pose estimator, and
 * displays visual layers on Field2d.
 */
public class Vision extends SubsystemBase {
  public static final double FIELD_LENGTH_METERS = 16.541748;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  private final Drive drivetrain;
  private final List<String> cameraNames;

  // Trackers for MegaTag1 (always wpiBlue)
  private final Map<String, Pose2d> mt1Poses = new HashMap<>();
  private final Map<String, List<PoseEstimate>> mt1MeasurementTracker = new HashMap<>();

  // Single main field widget with individual object trackers per camera
  private final Field2d visionField = new Field2d();
  private final Map<String, FieldObject2d> cameraFieldObjects = new HashMap<>();

  private final SendableChooser<String> disableChooser = new SendableChooser<>();

  private double timeDelay = 0.5; // Seconds window to keep measurements
  private int maxFrameCount = 50;
  private int rewindCaptureCounter = 0;

  /** Initializes Vision subsystem with a reference to the drivetrain and Limelight camera names. */
  public Vision(Drive drivetrain, String... cameraNames) {
    this.drivetrain = drivetrain;

    if (cameraNames != null && cameraNames.length > 0) {
      this.cameraNames = Arrays.asList(cameraNames);
    } else {
      this.cameraNames = List.of("limelight-shooter", "limelight-left", "limelight-right");
    }

    for (String name : this.cameraNames) {
      mt1MeasurementTracker.put(name, new ArrayList<>());
      mt1Poses.put(name, new Pose2d());
      cameraFieldObjects.put(name, visionField.getObject(name));
    }

    visionField.setRobotPose(new Pose2d());
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
  }

  public void setup() {
    // Standard MegaTag1 setup
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

  public void startRewind() {
    for (String name : List.of("limelight-left", "limelight-right")) {
      LimelightHelpers.setLimelightNTDouble(name, "rewind_enable_set", 1.0);
      LimelightHelpers.setLimelightNTDoubleArray(name, "capture_rewind", new double[] {0, 0});
    }
  }

  public void captureRewind(double durationSeconds) {
    rewindCaptureCounter++;
    for (String name : List.of("limelight-left", "limelight-right")) {
      LimelightHelpers.setLimelightNTDoubleArray(
          name, "capture_rewind", new double[] {rewindCaptureCounter, durationSeconds});
    }
  }

  public void captureRewind() {
    captureRewind(165.0);
  }

  /** Flips a pose 180 degrees across the field center (used when on Red Alliance). */
  public static Pose2d flipFieldPose(Pose2d pose) {
    return new Pose2d(
        FIELD_LENGTH_METERS - pose.getX(),
        FIELD_WIDTH_METERS - pose.getY(),
        pose.getRotation().plus(Rotation2d.kPi));
  }

  @Override
  public void periodic() {
    String disabledSelection = disableChooser.getSelected();
    List<Pose2d> validPosesThisCycle = new ArrayList<>();
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    for (String cameraName : cameraNames) {
      if ("all".equals(disabledSelection) || cameraName.equals(disabledSelection)) {
        continue;
      }

      // Retrieve estimates in WPILib Blue Alliance coordinates (botpose_wpiblue)
      PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

      // Process MegaTag1 Estimate
      if (LimelightHelpers.validPoseEstimate(mt1Estimate)) {
        Pose2d poseToAdd = isRedAlliance ? flipFieldPose(mt1Estimate.pose) : mt1Estimate.pose;
        Matrix<N3, N1> stdDevs = calculateStdDevs(mt1Estimate);
        drivetrain.addVisionMeasurement(poseToAdd, mt1Estimate.timestampSeconds, stdDevs);

        mt1Poses.put(cameraName, poseToAdd);
        mt1MeasurementTracker.get(cameraName).add(mt1Estimate);
        validPosesThisCycle.add(poseToAdd);

        FieldObject2d cameraObj = cameraFieldObjects.get(cameraName);
        if (cameraObj != null) {
          cameraObj.setPose(poseToAdd);
        }
      }

      cleanTracker(mt1MeasurementTracker.get(cameraName));
    }

    // Update main robot pose on Field2d widget
    if (!validPosesThisCycle.isEmpty()) {
      Pose2d avgPose = getAveragePose(validPosesThisCycle);
      visionField.setRobotPose(avgPose);
    } else {
      visionField.setRobotPose(drivetrain.getPose());
    }
  }

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

  private void cleanTracker(List<PoseEstimate> tracker) {
    if (tracker == null) return;
    double now = Timer.getFPGATimestamp();
    tracker.removeIf(m -> (now - m.timestampSeconds) > timeDelay);

    while (maxFrameCount > 0 && tracker.size() > maxFrameCount) {
      tracker.remove(0);
    }
  }

  public Pose2d getRecentMt1Pose(String limelightName) {
    return mt1Poses.getOrDefault(limelightName, new Pose2d());
  }

  public double getRecentFps(String limelightName) {
    List<PoseEstimate> tracker = mt1MeasurementTracker.get(limelightName);
    if (tracker == null || timeDelay <= 0) {
      return 0.0;
    }
    return tracker.size() / timeDelay;
  }

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

  public static Pose2d getAveragePose(Pose2d... poses) {
    return getAveragePose(Arrays.asList(poses));
  }
}
