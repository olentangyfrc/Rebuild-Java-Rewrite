package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static final BooleanEntry shooterOverrideEnabled =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getBooleanTopic("Shooter/OverrideEnabled")
          .getEntry(false);

  private static final DoubleEntry manualHoodAngleDegrees =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleTopic("Shooter/ManualHoodAngleDegrees")
          .getEntry(20.0);

  private static final DoubleEntry manualDrumRPM =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleTopic("Shooter/ManualDrumRPM")
          .getEntry(2000.0);

  private static final BooleanEntry passingEnabled =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getBooleanTopic("Shooter/Passing")
          .getEntry(false);

  private static final DoubleEntry passingScaleMultiplier =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleTopic("Shooter/PassingScaleMultiplier")
          .getEntry(1.0);

  static {
    shooterOverrideEnabled.setDefault(false);
    manualHoodAngleDegrees.setDefault(20.0);
    manualDrumRPM.setDefault(2000.0);
    passingEnabled.setDefault(false);
    passingScaleMultiplier.setDefault(1.0);
  }

  private TalonFX leftTopDrumLeader;
  private TalonFX leftBottomDrumFollower;
  private TalonFX rightTopDrumFollower;
  private TalonFX rightBottomDrumFollower;
  private TalonFX indexerFeeder;
  private TalonFX indexerTunnel;
  private TalonFX hoodMotor;
  private CANcoder hoodEncoder;

  private PIDController hoodPIDController;
  private ArmFeedforward hoodFFWController;
  private double hoodTargetAngle;
  private double targetDrumRpm = 0.0;

  private TalonFXConfiguration leftTopDrumLeaderConfig;
  private TalonFXConfiguration commonDrumFollowerConfig;
  private TalonFXConfiguration indexerFeederConfig;
  private TalonFXConfiguration indexerTunnelConfig;
  private TalonFXConfiguration hoodMotorConfig;

  private final int leftTopDrumLeaderCanId = 22;
  private final int leftBottomDrumFollowerCanId = 23;
  private final int rightTopDrumFollowerCanId = 20;
  private final int rightBottomDrumFollowerCanId = 21;
  private final int indexerFeederCanId = 24;
  private final int indexerTunnelCanId = 25;
  private final int hoodMotorCanId = 26;
  private final int hoodEncoderCanId = 47;

  // private double drumTargetVelocityTolerance = 1.0; // rps

  private double spinUpVelocity = 1500; // rpm
  private double maxdrumVelocity = 4000; // RPM

  private final boolean lowCeiling = false;

  public Shooter() {
    leftTopDrumLeader = new TalonFX(leftTopDrumLeaderCanId, "can0");
    leftBottomDrumFollower = new TalonFX(leftBottomDrumFollowerCanId, "can0");
    rightTopDrumFollower = new TalonFX(rightTopDrumFollowerCanId, "can0");
    rightBottomDrumFollower = new TalonFX(rightBottomDrumFollowerCanId, "can0");

    hoodMotor = new TalonFX(hoodMotorCanId, "can0");

    indexerFeeder = new TalonFX(indexerFeederCanId, "can0");
    indexerTunnel = new TalonFX(indexerTunnelCanId, "can0");

    hoodEncoder = new CANcoder(hoodEncoderCanId, "can0");

    hoodPIDController = new PIDController(5.2, 0, 0);
    hoodFFWController = new ArmFeedforward(0.04, 0.29, 0);
    hoodPIDController.setTolerance(Math.toRadians(1.5)); // min-max hood angle: 2 - 47
    hoodPIDController.setIZone(Math.toRadians(0.5));
    hoodPIDController.setSetpoint(Math.toRadians(3));
    hoodPIDController.reset();
  }

  public void init() {
    leftTopDrumLeaderConfig = new TalonFXConfiguration();
    leftTopDrumLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftTopDrumLeaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftTopDrumLeaderConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    leftTopDrumLeaderConfig.Slot0.kP = 0.21164 * 2;
    leftTopDrumLeaderConfig.Slot0.kI = 0.0;
    leftTopDrumLeaderConfig.Slot0.kD = 0.0;
    leftTopDrumLeaderConfig.Slot0.kS = 0.14158;
    leftTopDrumLeaderConfig.Slot0.kV = 0.12061;
    leftTopDrumLeaderConfig.Slot0.kA = 0.0038129;

    leftTopDrumLeader.getConfigurator().apply(leftTopDrumLeaderConfig, 0.25);
    // leftTopDrumLeader.getConfigurator().apply(new CurrentLimitsConfigs());

    commonDrumFollowerConfig = new TalonFXConfiguration();
    commonDrumFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftBottomDrumFollower.getConfigurator().apply(commonDrumFollowerConfig);
    leftBottomDrumFollower.setControl(
        new Follower(leftTopDrumLeaderCanId, MotorAlignmentValue.Aligned));

    rightTopDrumFollower.getConfigurator().apply(commonDrumFollowerConfig);
    rightTopDrumFollower.setControl(
        new Follower(leftTopDrumLeaderCanId, MotorAlignmentValue.Opposed));

    rightBottomDrumFollower.getConfigurator().apply(commonDrumFollowerConfig);
    rightBottomDrumFollower.setControl(
        new Follower(leftTopDrumLeaderCanId, MotorAlignmentValue.Opposed));

    indexerFeederConfig = new TalonFXConfiguration();
    indexerFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerFeederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexerFeederConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    indexerFeederConfig.Slot0.kP = 0.10865;
    indexerFeederConfig.Slot0.kI = 0.0;
    indexerFeederConfig.Slot0.kD = 0.0;
    indexerFeederConfig.Slot0.kS = 0.37921;
    indexerFeederConfig.Slot0.kV = 0.097873;
    indexerFeederConfig.Slot0.kA = 0.0029301;

    indexerFeeder.getConfigurator().apply(indexerFeederConfig, 0.25);

    indexerTunnelConfig = new TalonFXConfiguration();
    indexerTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerTunnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexerTunnelConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    indexerTunnelConfig.Slot0.kP = 0.17464;
    indexerTunnelConfig.Slot0.kI = 0.0;
    indexerTunnelConfig.Slot0.kD = 0.0;
    indexerTunnelConfig.Slot0.kS = 0.34955;
    indexerTunnelConfig.Slot0.kV = 0.09769;
    indexerTunnelConfig.Slot0.kA = 0.0029558;

    indexerTunnel.getConfigurator().apply(indexerTunnelConfig, 0.25);

    hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodMotor.getConfigurator().apply(hoodMotorConfig, 0.25);
  }

  public boolean isHoodAtSetpoint() {
    return hoodPIDController.atSetpoint();
  }

  // public boolean isHoodAtSetPoint() {
  //   return isHoodAtSetpoint();
  // }

  public void resetHood() {
    setHoodSetPoint(Math.toRadians(2));
  }

  public double getHoodAngle() {
    return Math.toRadians((hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360) + 31);
  }

  public boolean isDrumAtSpeed() {
    return leftTopDrumLeader.getClosedLoopError().getValueAsDouble() < 1.5;
  }
  // Set hood in radians from (2 - 47 Degrees)

  public void setHoodSetPoint(double hoodSetPoint) {
    hoodTargetAngle = MathUtil.clamp(hoodSetPoint, Math.toRadians(2), Math.toRadians(47));
  }

  public boolean isPassing() {
    return passingEnabled.get();
  }

  public double getDrumSpeedScale() {
    if (targetDrumRpm <= 0) {
      return 1.0;
    }
    double currentDrumRpm = leftTopDrumLeader.getVelocity().getValueAsDouble() * 60.0;
    double ratio = currentDrumRpm / targetDrumRpm;
    double baseScale = MathUtil.clamp(ratio, 0.0, 1.0);
    return baseScale * passingScaleMultiplier.get();
  }

  public void periodic() {
    hoodMotor.setControl(
        new com.ctre.phoenix6.controls.VoltageOut(
            hoodPIDController.calculate(getHoodAngle(), hoodTargetAngle)
                + hoodFFWController.calculate(getHoodAngle(), 0)));
    org.littletonrobotics.junction.Logger.recordOutput("Shooter/IsPassing", isPassing());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/DrumSpeedScale", getDrumSpeedScale());
  }

  public void spinUpDrum() {
    setDrumVelocity(spinUpVelocity);
  }

  // Set drum speed to a specific velocity in RPM
  public void setDrumVelocity(double velocity) {
    this.targetDrumRpm = velocity;
    if (!lowCeiling) {
      leftTopDrumLeader.setControl(
          new com.ctre.phoenix6.controls.VelocityVoltage(
              MathUtil.clamp((velocity / 60), 0, (maxdrumVelocity / 60))));
    } else if (lowCeiling) {
      leftTopDrumLeader.setControl(
          new com.ctre.phoenix6.controls.VelocityVoltage(
              MathUtil.clamp((velocity / 60), 0, ((maxdrumVelocity / 60) / 3))));
    }
  }

  // Sets Feeder PID velocity to 0
  public void HoldFeeder() {
    indexerFeeder.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(0));
  }

  // Sets Tunnel PID velocity to 0
  public void HoldTunnel() {
    indexerTunnel.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(0));
  }

  // Set Tunnel Velocity in RPM
  public void setTunnelVelocity(double velocity) {
    double scale = (isPassing() && velocity > 0) ? getDrumSpeedScale() : 1.0;
    indexerTunnel.setControl(
        new com.ctre.phoenix6.controls.VelocityVoltage((velocity * scale) / 60));
  }

  // Set Feeder Velocity in RPM
  public void setFeederVelocity(double velocity) {
    double scale = (isPassing() && velocity > 0) ? getDrumSpeedScale() : 1.0;
    indexerFeeder.setControl(
        new com.ctre.phoenix6.controls.VelocityVoltage((velocity * scale) / 60));
  }

  // Sets Voltage out for Tunnel to 0 (roll to stop)
  public void stopTunnel() {
    indexerTunnel.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
  }

  // Sets Voltage out for Feeder to 0 (roll to stop)
  public void stopFeeder() {
    indexerFeeder.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
  }

  // Sets all Voltages for indexer,feeder and drum to 0 (roll to stop)
  public void stop() {
    targetDrumRpm = 0.0;
    leftTopDrumLeader.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    indexerFeeder.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    indexerTunnel.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }

  public void shootForHub(double distanceMeters) {
    if (shooterOverrideEnabled.get()) {
      setHoodSetPoint(Math.toRadians(manualHoodAngleDegrees.get()));
      setDrumVelocity(manualDrumRPM.get());
    } else {
      ShooterUtil.ShooterParameters params = ShooterUtil.getInterpolatedValues(distanceMeters);
      setHoodSetPoint(params.hoodAngleRad());
      setDrumVelocity(params.flywheelRpm());
    }
  }
  public void pass(double distanceMeters) {
    if (shooterOverrideEnabled.get()) {
      setHoodSetPoint(Math.toRadians(manualHoodAngleDegrees.get()));
      setDrumVelocity(manualDrumRPM.get());
    } else {
      ShooterUtil.ShooterParameters params = ShooterUtil.getInterpolatedValuesPass(distanceMeters);
      setHoodSetPoint(params.hoodAngleRad());
      setDrumVelocity(params.flywheelRpm());
    }
  }


  public void startfeed() {
    setTunnelVelocity(5000);
    setFeederVelocity(5000);
  }

  public void waitforfeed() {
    setTunnelVelocity((40 * 60));
    setFeederVelocity((-1800));
  }

  public void stopfeed() {
    stopTunnel();
    stopFeeder();
  }
}
