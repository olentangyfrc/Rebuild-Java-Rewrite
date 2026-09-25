package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterUtil {

  public record ShooterParameters(double hoodAngleRad, double flywheelRpm) {}
  // Chase messed up our limelight configs and we need this to offset
  private static final double CHASE_FUDGE_FACTOR = 0.6578092;
  // This needs to be tuned (TO BE DONE)
  private static final double NEW_INDEXER_FUDGE_FACTOR = -40;

  private static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap angleMapPass = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap rpmMapPass = new InterpolatingDoubleTreeMap();
  static {
    // High-ceiling table points: { distance (m), angle (deg), rpm }
    double[][] highCeilingData = {
      {2.20 + CHASE_FUDGE_FACTOR, 10.5, 1540 + NEW_INDEXER_FUDGE_FACTOR},
      {2.99 + CHASE_FUDGE_FACTOR, 16.0, 1545 + NEW_INDEXER_FUDGE_FACTOR},
      {2.67 + CHASE_FUDGE_FACTOR, 13.0, 1530 + NEW_INDEXER_FUDGE_FACTOR},
      {2.45 + CHASE_FUDGE_FACTOR, 9.0, 1510 + NEW_INDEXER_FUDGE_FACTOR},
      {2.02 + CHASE_FUDGE_FACTOR, 7.0, 1495 + NEW_INDEXER_FUDGE_FACTOR},
      {1.83 + CHASE_FUDGE_FACTOR, 6.0, 1460 + NEW_INDEXER_FUDGE_FACTOR},
      {1.56 + CHASE_FUDGE_FACTOR, 5.0, 1410 + NEW_INDEXER_FUDGE_FACTOR},
      {1.31 + CHASE_FUDGE_FACTOR, 5.0, 1380 + NEW_INDEXER_FUDGE_FACTOR},
      {1.04 + CHASE_FUDGE_FACTOR, 5.0, 1330 + NEW_INDEXER_FUDGE_FACTOR},
      {0.96 + CHASE_FUDGE_FACTOR, 5.0, 1270 + NEW_INDEXER_FUDGE_FACTOR},
      {3.25 + CHASE_FUDGE_FACTOR, 16.0, 1605 + NEW_INDEXER_FUDGE_FACTOR},
      {3.50 + CHASE_FUDGE_FACTOR, 18.0, 1610 + NEW_INDEXER_FUDGE_FACTOR},
      {0.78 + CHASE_FUDGE_FACTOR, 2.0, 1250 + NEW_INDEXER_FUDGE_FACTOR},
      {3.75 + CHASE_FUDGE_FACTOR, 20.0, 1625 + NEW_INDEXER_FUDGE_FACTOR},
      {4.00 + CHASE_FUDGE_FACTOR, 21.0, 1650 + NEW_INDEXER_FUDGE_FACTOR},
      {4.30 + CHASE_FUDGE_FACTOR, 23.0, 1680 + NEW_INDEXER_FUDGE_FACTOR},
      {4.58 + CHASE_FUDGE_FACTOR, 23.5, 1750 + NEW_INDEXER_FUDGE_FACTOR},
      {5.29 + CHASE_FUDGE_FACTOR, 25.0, 1800 + NEW_INDEXER_FUDGE_FACTOR}
    };

    for (double[] point : highCeilingData) {
      double distanceMeters = point[0];
      double angleRadians = Math.toRadians(point[1]);
      double rpm = point[2];

      angleMap.put(distanceMeters, angleRadians);
      rpmMap.put(distanceMeters, rpm);
    }
  }
  static {
    // High-ceiling table points: { distance (m), angle (deg), rpm }
    double[][] highCeilingData = {
      {1.44 + CHASE_FUDGE_FACTOR, 2.04, 1210 + NEW_INDEXER_FUDGE_FACTOR},
      {1.64 + CHASE_FUDGE_FACTOR, 5, 1246 + NEW_INDEXER_FUDGE_FACTOR},
      {1.84 + CHASE_FUDGE_FACTOR, 5,1316 + NEW_INDEXER_FUDGE_FACTOR},
      {2.04 + CHASE_FUDGE_FACTOR, 5, 1348 + NEW_INDEXER_FUDGE_FACTOR},
      {2.24 + CHASE_FUDGE_FACTOR, 5.08, 1374 + NEW_INDEXER_FUDGE_FACTOR},
      {2.44 + CHASE_FUDGE_FACTOR, 5.82, 1411 + NEW_INDEXER_FUDGE_FACTOR},
      {2.64 + CHASE_FUDGE_FACTOR, 6.8, 1448 + NEW_INDEXER_FUDGE_FACTOR},
      {2.84 + CHASE_FUDGE_FACTOR, 10.15, 1495 + NEW_INDEXER_FUDGE_FACTOR},
      {3.04 + CHASE_FUDGE_FACTOR, 9.41, 1478 + NEW_INDEXER_FUDGE_FACTOR},
      {3.24 + CHASE_FUDGE_FACTOR, 11.4, 1482 + NEW_INDEXER_FUDGE_FACTOR},
      {3.44 + CHASE_FUDGE_FACTOR, 14.05, 1495 + NEW_INDEXER_FUDGE_FACTOR},
      {3.64 + CHASE_FUDGE_FACTOR, 15.93, 1504 + NEW_INDEXER_FUDGE_FACTOR},
      {3.84 + CHASE_FUDGE_FACTOR, 16, 1549 + NEW_INDEXER_FUDGE_FACTOR},
      {4.04 + CHASE_FUDGE_FACTOR, 17.06, 1567 + NEW_INDEXER_FUDGE_FACTOR},
      {4.24 + CHASE_FUDGE_FACTOR, 18.66, 1574 + NEW_INDEXER_FUDGE_FACTOR},
      {4.44 + CHASE_FUDGE_FACTOR, 20.13, 1588 + NEW_INDEXER_FUDGE_FACTOR},
      {4.64 + CHASE_FUDGE_FACTOR, 20.93, 1608 + NEW_INDEXER_FUDGE_FACTOR},
      {4.84 + CHASE_FUDGE_FACTOR, 22.21, 1628 + NEW_INDEXER_FUDGE_FACTOR},
      {5.04 + CHASE_FUDGE_FACTOR, 23.15, 1660 + NEW_INDEXER_FUDGE_FACTOR},
      {5.24 + CHASE_FUDGE_FACTOR, 23.5, 1710 + NEW_INDEXER_FUDGE_FACTOR},
      {5.44 + CHASE_FUDGE_FACTOR, 23.93, 1724 + NEW_INDEXER_FUDGE_FACTOR},
    };

    for (double[] point : highCeilingData) {
      double distanceMeters = point[0];
      double angleRadians = Math.toRadians(point[1]);
      double rpm = point[2];

      angleMapPass.put(distanceMeters, angleRadians);
      rpmMapPass.put(distanceMeters, rpm);
    }
  }

  private ShooterUtil() {}
  public static double getAngleBetweenPoses(Pose2d botPose2d, Pose2d targetPose2d){
    double y_diff = targetPose2d.getY() - botPose2d.getY();
    double x_diff = targetPose2d.getX() - botPose2d.getX();
    return Math.atan2(y_diff, x_diff);
  }
  public static Rotation2d getPassAngle(Pose2d pose){
    double fieldWidth = 8.0137;
    if(pose.getX() < 11.2){
      if (pose.getY() < 3.45){
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(2.00, 1.00, new Rotation2d(0)))+Math.PI);
      return result;
    }
      else if (pose.getY() > 4.65){
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(2.00, fieldWidth - 1, new Rotation2d(0)))+Math.PI);
      return result;
    }
    else if (pose.getY() < 4){
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(3.2, 1.4, new Rotation2d(0)))+Math.PI);
      return result;      
    }
    else {
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(3.2, 6.5, new Rotation2d(0)))+Math.PI);
      return result;      
    }
    }
    else{
      if (pose.getY() < 3.45){
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(5.67, 2.5, new Rotation2d(0)))+Math.PI);
      return result;
    }
    else if (pose.getY() > 4.65){
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(5.67, fieldWidth-2.5, new Rotation2d(0)))+Math.PI);
      return result;      
    }
    else if (pose.getY() < 4){
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(10, 1.4, new Rotation2d(0)))+Math.PI);
      return result;      
    }
    else {
      Rotation2d result = new Rotation2d(getAngleBetweenPoses(pose, new Pose2d(10, 6.5, new Rotation2d(0)))+Math.PI);
      return result;      
    }
    }
  }
  public static double getPassDistance(Pose2d pose){
    double fieldWidth = 8.0137;
    if (pose.getX() < 11.2){
      if (pose.getY() < 3.45){
      return pose.getTranslation().getDistance(new Translation2d(2, 1));
    }
    else if (pose.getY() > 4.65){
      return pose.getTranslation().getDistance(new Translation2d(2, 1));
    }
    else if (pose.getY() < 4){
      return pose.getTranslation().getDistance(new Translation2d(3.2, 1.4));
    }
    else{
      return pose.getTranslation().getDistance(new Translation2d(3.2, 6.5));
    }
    }
    else{
      if (pose.getY() < 3.45){
        return pose.getTranslation().getDistance(new Translation2d(5.67, 2.5));
      }
      else if (pose.getY() > 4.65){
        return pose.getTranslation().getDistance(new Translation2d(5.67, fieldWidth - 2.5));
      }
      else if (pose.getY() < 4){
        return pose.getTranslation().getDistance(new Translation2d(10, 1.4));
      }
      else {
        return pose.getTranslation().getDistance(new Translation2d(10, 6.5));
      }
    }
  }

  public static ShooterParameters getInterpolatedValues(double distanceMeters) {
    double angle = angleMap.get(distanceMeters);
    double rpm = Math.floor(rpmMap.get(distanceMeters));
    return new ShooterParameters(angle, rpm);
  }
  public static ShooterParameters getInterpolatedValuesPass(double distanceMeters) {
    double angle = angleMapPass.get(distanceMeters);
    double rpm = Math.floor(rpmMapPass.get(distanceMeters));
    return new ShooterParameters(angle, rpm);
  }

  public static double getInterpolatedAngle(double distanceMeters) {
    return angleMap.get(distanceMeters);
  }

  public static double getInterpolatedRPM(double distanceMeters) {
    return Math.floor(rpmMap.get(distanceMeters));
  }
}
