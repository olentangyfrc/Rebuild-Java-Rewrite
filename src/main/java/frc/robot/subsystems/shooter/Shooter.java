package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum ShooterState {
    IDLE,
    WARM_UP,
    SPIN_UP,
    SHOOT,
    EJECT,
    COOLDOWN
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final PIDController hoodPid;
  private final ArmFeedforward hoodFf;

  private static final double MAX_SHOOTER_RPM = 4000;
  private static final double MIN_HOOD_ANGLE = Math.toRadians(2);
  private static final double MAX_HOOD_ANGLE = Math.toRadians(47);

  private ShooterState state = ShooterState.IDLE;
  private double targetRpm = 0.0;
  private double targetIndexer1Rps = 0.0;
  private double targetIndexer2Rps = 0.0;
  private double targetHoodAngle = Math.toRadians(2);

  private double shootRpm = 1500;
  private double shootAngle = Math.toRadians(20);
  private double warmUpRpm = 1000;
  private boolean passing = false;

  private final Timer shotTimer = new Timer();
  private final Timer initialFeedTimer = new Timer();
  private final Timer jamDetectionTimer = new Timer();
  private final Timer dryFireTimer = new Timer();

  public Shooter(ShooterIO io) {
    this.io = io;
    hoodFf = new ArmFeedforward(0.04, 0.29, 0.0);
    hoodPid = new PIDController(5.2, 0.0, 0.0);
    hoodPid.setTolerance(Math.toRadians(0.5));
    hoodPid.setIZone(Math.toRadians(3));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    handleStateLogic();
    Logger.processInputs("Shooter", inputs);

    // Ceiling safety logic from python
    double theta = Math.toDegrees(targetHoodAngle);
    double maxRpm = 0.578583 * (theta * theta) - (12.35148 * theta) + 1300;
    maxRpm = Math.min(maxRpm, MAX_SHOOTER_RPM);

    // In python they had a toggle for low ceiling, defaulting to true for safety
    double clampedRpm = MathUtil.clamp(targetRpm, -maxRpm, maxRpm);

    double clampedHood = MathUtil.clamp(targetHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
    hoodPid.setSetpoint(clampedHood);

    double hoodVolts =
        hoodPid.calculate(inputs.hoodAngleRads) + hoodFf.calculate(inputs.hoodAngleRads, 0);

    if (inputs.hoodAngleRads < MAX_HOOD_ANGLE + Math.toRadians(2)) {
      if (hoodPid.atSetpoint()) hoodPid.reset();
      io.setHoodVoltage(hoodVolts);
    } else {
      io.setHoodVoltage(-1);
    }

    io.setShooterVelocity(clampedRpm / 60.0);
    io.setIndexerVelocity(targetIndexer1Rps, targetIndexer2Rps);

    inputs.state = state.name();
  }

  private void handleStateLogic() {
    switch (state) {
      case IDLE:
        targetRpm = 0;
        targetIndexer1Rps = 0;
        targetIndexer2Rps = 0;
        targetHoodAngle = Math.toRadians(2);
        break;

      case WARM_UP:
        targetRpm = warmUpRpm;
        targetIndexer1Rps = 0;
        targetIndexer2Rps = 0;
        targetHoodAngle = Math.toRadians(2);
        break;

      case SPIN_UP:
        targetIndexer1Rps = -30;
        targetIndexer2Rps = -5;
        targetRpm = shootRpm;
        targetHoodAngle = shootAngle;

        if (initialFeedTimer.get() == 0) initialFeedTimer.start();

        if (atTargetSpeed()) {
          state = ShooterState.SHOOT;
          dryFireTimer.restart();
          jamDetectionTimer.stop();
          jamDetectionTimer.reset();
        }
        break;

      case SHOOT:
        // Jam detection
        if (inputs.indexer1TorqueCurrent > 100 && inputs.indexer1VelocityRps < 70) {
          if (jamDetectionTimer.get() == 0) jamDetectionTimer.start();
          else if (jamDetectionTimer.get() > 0.35) {
            state = ShooterState.SPIN_UP;
          }
        } else {
          jamDetectionTimer.stop();
          jamDetectionTimer.reset();
          initialFeedTimer.stop();
          initialFeedTimer.reset();

          targetIndexer1Rps = passing ? 30 : 60;
          targetIndexer2Rps = passing ? 35 : 70;
          targetRpm = shootRpm;
          targetHoodAngle = shootAngle;

          if (shotTimer.get() == 0) shotTimer.start();
        }
        break;

      case EJECT:
        targetIndexer1Rps = -50;
        targetIndexer2Rps = -30;
        targetRpm = 0;
        break;

      case COOLDOWN:
        targetIndexer1Rps = 0;
        targetIndexer2Rps = 30;
        if (shotTimer.get() == 0) shotTimer.start();
        if (shotTimer.get() > 1.0) {
          targetRpm = 0;
          state = ShooterState.IDLE;
          shotTimer.stop();
          shotTimer.reset();
        }
        break;
    }
  }

  public void setShootParams(double rpm, double angle) {
    this.shootRpm = rpm;
    this.shootAngle = angle;
  }

  public void setState(ShooterState newState) {
    this.state = newState;
  }

  public ShooterState getState() {
    return this.state;
  }

  public Command idleCommand() {
    return runOnce(() -> state = ShooterState.IDLE).withName("ShooterIdle");
  }

  public Command warmUpCommand() {
    return runOnce(() -> state = ShooterState.WARM_UP).withName("ShooterWarmUp");
  }

  public Command spinUpCommand() {
    return runOnce(
            () -> {
              state = ShooterState.SPIN_UP;
              passing = false;
            })
        .withName("ShooterSpinUp");
  }

  public Command shootCommand() {
    return runOnce(
            () -> {
              state = ShooterState.SPIN_UP;
              passing = false;
            })
        .withName("ShooterShoot");
  }

  public Command passCommand() {
    return runOnce(
            () -> {
              state = ShooterState.SPIN_UP;
              passing = true;
            })
        .withName("ShooterPass");
  }

  public Command ejectCommand() {
    return runOnce(() -> state = ShooterState.EJECT).withName("ShooterEject");
  }

  public Command cooldownCommand() {
    return runOnce(() -> state = ShooterState.COOLDOWN).withName("ShooterCooldown");
  }

  public boolean atTargetSpeed() {
    if (targetRpm == 0) return false;
    double currentRpm = inputs.leftBottomVelocityRps * 60.0;
    return Math.abs(currentRpm - targetRpm) / targetRpm < 0.015;
  }
}
