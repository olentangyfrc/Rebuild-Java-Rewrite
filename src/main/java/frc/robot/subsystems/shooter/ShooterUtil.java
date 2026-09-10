package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterUtil {

  public record ShooterParameters(double hoodAngleRad, double flywheelRpm) {}
  // Chase messed up our limelight configs and we need this to offset
  private static final double CHASE_FUDGE_FACTOR = 0.6578092;
  // This needs to be tuned (TO BE DONE)
  private static final double NEW_INDEXER_FUDGE_FACTOR = -40;

  private static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

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

  private ShooterUtil() {}

  public static ShooterParameters getInterpolatedValues(double distanceMeters) {
    double angle = angleMap.get(distanceMeters);
    double rpm = Math.floor(rpmMap.get(distanceMeters));
    return new ShooterParameters(angle, rpm);
  }

  public static double getInterpolatedAngle(double distanceMeters) {
    return angleMap.get(distanceMeters);
  }

  public static double getInterpolatedRPM(double distanceMeters) {
    return Math.floor(rpmMap.get(distanceMeters));
  }
}
