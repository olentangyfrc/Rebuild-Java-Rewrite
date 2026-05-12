package frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Interpolation {
  public static class DataPoint {
    public final double distance;
    public final double angleDegrees;
    public final double rpm;

    public DataPoint(double distance, double angleDegrees, double rpm) {
      this.distance = distance;
      this.angleDegrees = angleDegrees;
      this.rpm = rpm;
    }
  }

  private final List<DataPoint> dataPoints = new ArrayList<>();
  private static final double CHASE_FUDGE_FACTOR = 0.6578092;
  private static final double NEW_INDEXER_FUDGE_FACTOR = -40;

  public Interpolation() {
    // High ceiling data from data.py
    dataPoints.add(new DataPoint(2.20 + CHASE_FUDGE_FACTOR, 10.5, 1540 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(2.99 + CHASE_FUDGE_FACTOR, 16, 1545 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(2.67 + CHASE_FUDGE_FACTOR, 13, 1530 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(2.45 + CHASE_FUDGE_FACTOR, 9, 1510 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(2.02 + CHASE_FUDGE_FACTOR, 7, 1495 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(1.83 + CHASE_FUDGE_FACTOR, 6, 1460 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(1.56 + CHASE_FUDGE_FACTOR, 5, 1410 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(1.31 + CHASE_FUDGE_FACTOR, 5, 1380 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(1.04 + CHASE_FUDGE_FACTOR, 5, 1330 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(0.96 + CHASE_FUDGE_FACTOR, 5, 1270 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(3.25 + CHASE_FUDGE_FACTOR, 16, 1605 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(3.50 + CHASE_FUDGE_FACTOR, 18, 1610 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(0.78 + CHASE_FUDGE_FACTOR, 2, 1250 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(3.75 + CHASE_FUDGE_FACTOR, 20, 1625 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(4.00 + CHASE_FUDGE_FACTOR, 21, 1650 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(4.30 + CHASE_FUDGE_FACTOR, 23, 1680 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(4.58 + CHASE_FUDGE_FACTOR, 23.5, 1750 + NEW_INDEXER_FUDGE_FACTOR));
    dataPoints.add(new DataPoint(5.29 + CHASE_FUDGE_FACTOR, 25, 1800 + NEW_INDEXER_FUDGE_FACTOR));

    dataPoints.sort(Comparator.comparingDouble(p -> p.distance));
  }

  public double[] getInterpolatedValues(double distance) {
    if (dataPoints.isEmpty()) return null;

    if (distance <= dataPoints.get(0).distance) {
      return new double[] {dataPoints.get(0).angleDegrees, dataPoints.get(0).rpm};
    }
    if (distance >= dataPoints.get(dataPoints.size() - 1).distance) {
      return new double[] {
        dataPoints.get(dataPoints.size() - 1).angleDegrees,
        dataPoints.get(dataPoints.size() - 1).rpm
      };
    }

    for (int i = 0; i < dataPoints.size() - 1; i++) {
      DataPoint p1 = dataPoints.get(i);
      DataPoint p2 = dataPoints.get(i + 1);

      if (distance >= p1.distance && distance <= p2.distance) {
        double t = (distance - p1.distance) / (p2.distance - p1.distance);
        double angle = p1.angleDegrees + t * (p2.angleDegrees - p1.angleDegrees);
        double rpm = p1.rpm + t * (p2.rpm - p1.rpm);
        return new double[] {angle, rpm};
      }
    }

    return null;
  }
}
