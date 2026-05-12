package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final PIDController hoodPid;
  private final ArmFeedforward hoodFf;

  private static final double MIN_HOOD_ANGLE = Math.toRadians(2);
  private static final double MAX_HOOD_ANGLE = Math.toRadians(47);

  private double targetRpm = 0.0;
  private double targetIndexer1Rps = 0.0;
  private double targetIndexer2Rps = 0.0;
  private double targetHoodAngle = Math.toRadians(2);

  private double shootRpm = 1500;
  private double shootAngle = Math.toRadians(20);

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
    Logger.processInputs("Shooter", inputs);

    double clampedHood = MathUtil.clamp(targetHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
    hoodPid.setSetpoint(clampedHood);

    double hoodVolts =
        hoodPid.calculate(inputs.hoodAngleRads) + hoodFf.calculate(inputs.hoodAngleRads, 0);
    if (inputs.hoodAngleRads < MAX_HOOD_ANGLE + Math.toRadians(2)) {
      io.setHoodVoltage(hoodVolts);
    } else {
      io.setHoodVoltage(-1);
    }

    io.setShooterVelocity(targetRpm / 60.0);
    io.setIndexerVelocity(targetIndexer1Rps, targetIndexer2Rps);
  }

  public void setGoal(double rpm, double indexer1, double indexer2, double angle) {
    this.targetRpm = rpm;
    this.targetIndexer1Rps = indexer1;
    this.targetIndexer2Rps = indexer2;
    this.targetHoodAngle = angle;
  }

  public void setShootParams(double rpm, double angle) {
    this.shootRpm = rpm;
    this.shootAngle = angle;
  }

  public Command idleCommand() {
    return run(() -> setGoal(0, 0, 0, Math.toRadians(2))).withName("ShooterIdle");
  }

  public Command warmUpCommand() {
    return run(() -> setGoal(1000, 0, 0, Math.toRadians(2))).withName("ShooterWarmUp");
  }

  public Command spinUpCommand() {
    return run(() -> setGoal(shootRpm, -30, -5, shootAngle)).withName("ShooterSpinUp");
  }

  public Command shootCommand() {
    return run(() -> setGoal(shootRpm, 60, 70, shootAngle)).withName("ShooterShoot");
  }

  public Command passCommand() {
    return run(() -> setGoal(shootRpm, 30, 35, shootAngle)).withName("ShooterPass");
  }

  public Command ejectCommand() {
    return run(() -> setGoal(0, -50, -30, Math.toRadians(2))).withName("ShooterEject");
  }

  public boolean atTargetSpeed() {
    if (targetRpm == 0) return false;
    double currentRpm = inputs.leftBottomVelocityRps * 60.0;
    return Math.abs(currentRpm - targetRpm) / targetRpm < 0.015;
  }
}
