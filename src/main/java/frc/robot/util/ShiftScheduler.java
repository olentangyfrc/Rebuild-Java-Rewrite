package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * ShiftScheduler handles FRC REBUILD game shift scheduling, hub active state, time remaining in
 * shifts, match time, FMS auto result monitoring, and driver rumble alerts.
 */
public final class ShiftScheduler {
  private static final Map<String, Double> SHIFT_TO_TIME_MAP =
      Map.of(
          "Shift 1", 130.0,
          "Shift 2", 105.0,
          "Shift 3", 80.0,
          "Shift 4", 55.0,
          "Endgame", 30.0);

  private static boolean initialized = false;
  private static boolean wonAuto = false;
  private static Set<String> activeShifts = new HashSet<>();
  private static List<Double> activeTimes = new ArrayList<>();
  private static int currentIndex = 0;

  // Controllers for vibration alerts
  private static CommandXboxController auxController = null;
  private static CommandXboxController driverController = null;

  // Rumble tracking state
  private static String lastShift = "";
  private static boolean warned7Sec = false;
  private static double rumbleEndTime = 0.0;
  private static double rumbleIntensity = 0.0;
  private static boolean isFeedingActive = false;

  private ShiftScheduler() {
    throw new UnsupportedOperationException("Utility class - do not instantiate");
  }

  /** Sets whether the shooter feeder is actively feeding fuel (triggers controller vibration). */
  public static void setFeedingActive(boolean active) {
    isFeedingActive = active;
  }

  /** Returns whether shooter feeder is actively feeding. */
  public static boolean isFeedingActive() {
    return isFeedingActive;
  }

  /** Registers the auxiliary driver controller for shift vibration alerts. */
  public static void setAuxController(CommandXboxController controller) {
    auxController = controller;
  }

  /** Registers the primary driver controller for shift vibration alerts. */
  public static void setDriverController(CommandXboxController controller) {
    driverController = controller;
  }

  /**
   * Filters the current robot alliance from DriverStation.
   *
   * @return "r" for Red alliance, "b" for Blue alliance, or "" if unassigned.
   */
  private static String filterCurrentAlliance() {
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      return "";
    }
    return allianceOpt.get() == DriverStation.Alliance.Red ? "r" : "b";
  }

  /** Reads FMS game specific message to schedule shift active times and determine auto winner. */
  public static void scheduleShiftTimes() {
    String msg = DriverStation.getGameSpecificMessage().toLowerCase().trim();
    String currentAlliance = filterCurrentAlliance();

    if (!("r".equals(msg) || "b".equals(msg)) || currentAlliance.isEmpty()) {
      return;
    }

    initialized = true;
    wonAuto = currentAlliance.equals(msg);

    if (wonAuto) {
      activeShifts = Set.of("Shift 2", "Shift 4", "Endgame");
    } else {
      activeShifts = Set.of("Shift 1", "Shift 3", "Endgame");
    }

    activeTimes = new ArrayList<>();
    for (String shift : activeShifts) {
      if (SHIFT_TO_TIME_MAP.containsKey(shift)) {
        activeTimes.add(SHIFT_TO_TIME_MAP.get(shift));
      }
    }
    activeTimes.sort(Collections.reverseOrder());
    currentIndex = 0;
  }

  /**
   * Gets the current shift phase based on match time and autonomous state.
   *
   * @return Current shift string ("Auton", "Shift 1", "Shift 2", "Shift 3", "Shift 4", "Endgame",
   *     or "Transition Shift").
   */
  public static String getCurrentShift() {
    if (DriverStation.isAutonomous()) {
      return "Auton";
    }

    double time = DriverStation.getMatchTime();

    if (time <= 30.0) {
      return "Endgame";
    } else if (time <= 55.0) {
      return "Shift 4";
    } else if (time <= 80.0) {
      return "Shift 3";
    } else if (time <= 105.0) {
      return "Shift 2";
    } else if (time <= 130.0) {
      return "Shift 1";
    }
    return "Transition Shift";
  }

  /**
   * Checks whether our alliance's Hub is currently active.
   *
   * @return True if active, false otherwise.
   */
  public static boolean isHubActive() {
    if (DriverStation.isAutonomous()) {
      return true;
    }
    if (!initialized) {
      return false;
    }
    return activeShifts.contains(getCurrentShift());
  }

  /**
   * Calculates time remaining in the current shift (time until next shift transition).
   *
   * @return Seconds remaining in the current shift.
   */
  public static double getTimeToNextShift() {
    if (!initialized) {
      return -1.0;
    }

    String shift = getCurrentShift();
    double matchTime = DriverStation.getMatchTime();

    if ("Endgame".equals(shift)) {
      return Math.max(0.0, matchTime);
    } else if ("Transition Shift".equals(shift)) {
      return Math.max(0.0, matchTime - 130.0);
    } else if ("Auton".equals(shift)) {
      return Math.max(0.0, matchTime);
    } else {
      Double shiftStartTime = SHIFT_TO_TIME_MAP.get(shift);
      if (shiftStartTime == null) {
        return 0.0;
      }
      double shiftEndTime = shiftStartTime - 25.0;
      return Math.max(0.0, matchTime - shiftEndTime);
    }
  }

  /**
   * Checks if an active shift is starting within the specified margin seconds.
   *
   * @param marginSeconds Warning margin in seconds.
   * @return True if active shift is starting within margin.
   */
  public static boolean checkActiveShiftUpcoming(double marginSeconds) {
    if (!initialized || activeTimes.isEmpty()) {
      return false;
    }

    double currentTime = DriverStation.getMatchTime();

    if (currentIndex < activeTimes.size() && currentTime <= activeTimes.get(currentIndex)) {
      if (currentIndex < activeTimes.size() - 1) {
        currentIndex++;
      }
    }

    if (currentIndex < activeTimes.size()) {
      return (currentTime - activeTimes.get(currentIndex)) <= marginSeconds;
    }

    return false;
  }

  /**
   * Checks if a shift change is upcoming within margin seconds.
   *
   * @param marginSeconds Warning margin in seconds.
   * @return True if shift change is upcoming.
   */
  public static boolean checkShiftChangeUpcoming(double marginSeconds) {
    if ("Endgame".equals(getCurrentShift())) {
      return false;
    }
    return getTimeToNextShift() <= marginSeconds;
  }

  /** Returns whether our alliance won auto according to FMS. */
  public static boolean hasWonAuto() {
    return wonAuto;
  }

  /** Returns human readable string for Auto Result ("WON", "LOST", or "PENDING"). */
  public static String getAutoResult() {
    if (!initialized) {
      return "PENDING";
    }
    return wonAuto ? "WON" : "LOST";
  }

  /** Returns whether ShiftScheduler has received FMS game message. */
  public static boolean isInitialized() {
    return initialized;
  }

  /**
   * Periodic update method to be called in Robot.robotPeriodic(). Updates shift initialization,
   * logs telemetries, and triggers driver vibration alerts.
   */
  public static void periodic() {
    if (!initialized) {
      scheduleShiftTimes();
    }

    double matchTime = Math.max(0.0, DriverStation.getMatchTime());
    double timeToNextShift = getTimeToNextShift();
    String currentShift = getCurrentShift();
    boolean hubActive = isHubActive();
    String autoResult = getAutoResult();
    boolean shiftUpcoming = checkShiftChangeUpcoming(7.0);

    // Vibration triggers for 7-second warning and shift swap events
    double now = Timer.getFPGATimestamp();
    if (DriverStation.isTeleopEnabled() && initialized) {
      // 7-second shift change alert
      if (!"Endgame".equals(currentShift)) {
        if (timeToNextShift <= 7.0 && timeToNextShift > 0.0 && !warned7Sec) {
          warned7Sec = true;
          rumbleEndTime = now + 0.6; // 0.6 second pulse warning at 7s
          rumbleIntensity = 0.45; // 45% vibration intensity (< 70%)
        }
      }

      // Shift swap event trigger
      if (!lastShift.isEmpty() && !currentShift.equals(lastShift)) {
        warned7Sec = false;
        rumbleEndTime = now + 0.8; // 0.8 second pulse on swap
        rumbleIntensity = 0.65; // 65% vibration intensity (< 70%)
      }
    } else {
      warned7Sec = false;
    }
    lastShift = currentShift;

    // Apply rumble to controllers (ONLY during Teleop; strictly disabled in Autonomous)
    double currentRumble = 0.0;
    if (DriverStation.isTeleopEnabled()) {
      if (now < rumbleEndTime) {
        currentRumble = rumbleIntensity;
      } else if (isFeedingActive) {
        currentRumble = 0.50; // 50% vibration intensity (< 70%) while feeding
      }
    } else {
      rumbleEndTime = 0.0;
      rumbleIntensity = 0.0;
    }

    if (auxController != null) {
      auxController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, currentRumble);
    }
    if (driverController != null) {
      driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, currentRumble);
    }

    // AdvantageKit Logging
    Logger.recordOutput("ShiftScheduler/MatchTimeRemaining", matchTime);
    Logger.recordOutput("ShiftScheduler/TimeToNextShift", timeToNextShift);
    Logger.recordOutput("ShiftScheduler/CurrentShift", currentShift);
    Logger.recordOutput("ShiftScheduler/HubActive", hubActive);
    Logger.recordOutput("ShiftScheduler/AutoResult", autoResult);
    Logger.recordOutput("ShiftScheduler/WonAuto", wonAuto);
    Logger.recordOutput("ShiftScheduler/ShiftChangeUpcoming", shiftUpcoming);

    // SmartDashboard / NetworkTables entries for Elastic Dashboard
    SmartDashboard.putNumber("ShiftScheduler/MatchTimeRemaining", matchTime);
    SmartDashboard.putNumber("ShiftScheduler/TimeToNextShift", timeToNextShift);
    SmartDashboard.putString("ShiftScheduler/CurrentShift", currentShift);
    SmartDashboard.putBoolean("ShiftScheduler/HubActive", hubActive);
    SmartDashboard.putString("ShiftScheduler/AutoResult", autoResult);
    SmartDashboard.putBoolean("ShiftScheduler/WonAuto", wonAuto);
    SmartDashboard.putBoolean("ShiftScheduler/ShiftChangeUpcoming", shiftUpcoming);

    // Top-level aliases for convenient Elastic Dashboard widgets
    SmartDashboard.putNumber("Match Time", matchTime);
    SmartDashboard.putNumber("Time to Next Shift", timeToNextShift);
    SmartDashboard.putString("Current Shift", currentShift);
    SmartDashboard.putBoolean("Hub Active", hubActive);
    SmartDashboard.putString("Auto Result", autoResult);
  }

  /**
   * Sets up an Elastic / Shuffleboard tab with formatted widgets for the REBUILD match & shifts.
   */
  public static void setupElasticTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("REBUILD Game");

    // Match Time Remaining (Big Display)
    tab.addNumber("Match Time Remaining", () -> Math.max(0.0, DriverStation.getMatchTime()))
        .withPosition(0, 0)
        .withSize(2, 2);

    // Time to Next Shift
    tab.addNumber("Time to Next Shift", ShiftScheduler::getTimeToNextShift)
        .withPosition(2, 0)
        .withSize(2, 2);

    // Hub Active Status Boolean Box
    tab.addBoolean("Hub Active", ShiftScheduler::isHubActive).withPosition(4, 0).withSize(2, 2);

    // Auto Result (WON / LOST / PENDING)
    tab.addString("Auto Result", ShiftScheduler::getAutoResult).withPosition(6, 0).withSize(2, 1);

    // Current Shift Name
    tab.addString("Current Shift", ShiftScheduler::getCurrentShift)
        .withPosition(6, 1)
        .withSize(2, 1);

    // Shift Change Warning Indicator (7s)
    tab.addBoolean("Shift Change Soon (7s)", () -> checkShiftChangeUpcoming(7.0))
        .withPosition(8, 0)
        .withSize(2, 2);
  }
}
