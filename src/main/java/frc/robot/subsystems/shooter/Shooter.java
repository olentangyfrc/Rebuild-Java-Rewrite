package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
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
  private double hoodSetPoint;

  private TalonFXConfiguration leftTopDrumLeaderConfig;
  private TalonFXConfiguration commonDrumFollowerConfig;
  private TalonFXConfiguration indexerFeederConfig;
  private TalonFXConfiguration indexerTunnelConfig;
  private TalonFXConfiguration hoodMotorConfig;
  private CANcoderConfiguration hoodEncoderConfig;

  private final int leftTopDrumLeaderCanId = 22;
  private final int leftBottomDrumFollowerCanId = 23;
  private final int rightTopDrumFollowerCanId = 20;
  private final int rightBottomDrumFollowerCanId = 21;
  private final int indexerFeederCanId = 24;
  private final int indexerTunnelCanId = 25;
  private final int hoodMotorCanId = 26;
  private final int hoodEncoderCanId = 47;

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
    hoodPIDController.setTolerance(Math.toRadians(0.5)); // min-max hood angle: 2 - 47
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

  public double getHoodAngle() {
    return Math.toRadians((hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360) + 31);
  }

  public void setHoodSetPoint(double hoodSetPoint) {
    hoodTargetAngle = MathUtil.clamp(hoodSetPoint, Math.toRadians(2), Math.toRadians(47));
  }

  public void periodic() {
    hoodMotor.setControl(
        new com.ctre.phoenix6.controls.VoltageOut(
            hoodPIDController.calculate(getHoodAngle(), hoodTargetAngle)
                + hoodFFWController.calculate(getHoodAngle(), 0)));
  }

  public void spinUp() {
    System.out.println("Spin Upping");
    leftTopDrumLeader.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(15.0));
    indexerFeeder.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(50.0));
    indexerTunnel.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(30.0));
  }

  public void stop() {
    System.out.println("Spin Downing");
    leftTopDrumLeader.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    indexerFeeder.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    indexerTunnel.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }
}
