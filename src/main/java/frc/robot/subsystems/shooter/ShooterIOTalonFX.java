package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftBottom;
  private final TalonFX leftTop;
  private final TalonFX rightBottom;
  private final TalonFX rightTop;

  private final TalonFX indexer1;
  private final TalonFX indexer2;

  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;

  private final StatusSignal<AngularVelocity> leftBottomVelocity;
  private final StatusSignal<AngularVelocity> leftTopVelocity;
  private final StatusSignal<AngularVelocity> rightBottomVelocity;
  private final StatusSignal<AngularVelocity> rightTopVelocity;

  private final StatusSignal<AngularVelocity> indexer1Velocity;
  private final StatusSignal<AngularVelocity> indexer2Velocity;
  private final StatusSignal<Angle> hoodPosition;

  private final MotionMagicVelocityVoltage shooterVelocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final MotionMagicVelocityVoltage indexerVelocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ShooterIOTalonFX() {
    leftBottom = new TalonFX(23, "can0");
    leftTop = new TalonFX(22, "can0");
    rightBottom = new TalonFX(21, "can0");
    rightTop = new TalonFX(20, "can0");

    indexer1 = new TalonFX(24, "can0");
    indexer2 = new TalonFX(25, "can0");

    hoodMotor = new TalonFX(26, "can0");
    hoodEncoder = new CANcoder(47, "can0");

    // Shooter config
    TalonFXConfiguration shootConfig = new TalonFXConfiguration();
    shootConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shootConfig.Slot0.kP = 0.21063;
    shootConfig.Slot0.kS = 0.16149;
    shootConfig.Slot0.kV = 0.13707;
    shootConfig.Slot0.kA = 0.042849;
    shootConfig.MotionMagic.MotionMagicAcceleration = 9999;

    leftBottom.getConfigurator().apply(shootConfig);
    leftTop.getConfigurator().apply(shootConfig);
    rightBottom.getConfigurator().apply(shootConfig);
    rightTop.getConfigurator().apply(shootConfig);

    leftTop.setControl(new Follower(23, MotorAlignmentValue.Aligned));
    rightBottom.setControl(new Follower(23, MotorAlignmentValue.Opposed));
    rightTop.setControl(new Follower(23, MotorAlignmentValue.Opposed));

    // Indexer config
    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    indexerConfig.Slot0.kP = 0.10865;
    indexerConfig.Slot0.kS = 0.37921;
    indexerConfig.Slot0.kV = 0.097873;
    indexerConfig.Slot0.kA = 0.0029301;
    indexer1.getConfigurator().apply(indexerConfig);

    TalonFXConfiguration indexer2Config = new TalonFXConfiguration();
    indexer2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexer2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexer2Config.Slot0.kP = 0.17464;
    indexer2Config.Slot0.kS = 0.34955;
    indexer2Config.Slot0.kV = 0.09769;
    indexer2Config.Slot0.kA = 0.0029558;
    indexer2.getConfigurator().apply(indexer2Config);

    // Hood Config
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodMotor.getConfigurator().apply(hoodConfig);

    leftBottomVelocity = leftBottom.getVelocity();
    leftTopVelocity = leftTop.getVelocity();
    rightBottomVelocity = rightBottom.getVelocity();
    rightTopVelocity = rightTop.getVelocity();

    indexer1Velocity = indexer1.getVelocity();
    indexer2Velocity = indexer2.getVelocity();
    hoodPosition = hoodEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftBottomVelocity,
        leftTopVelocity,
        rightBottomVelocity,
        rightTopVelocity,
        indexer1Velocity,
        indexer2Velocity,
        hoodPosition);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftBottomVelocity,
        leftTopVelocity,
        rightBottomVelocity,
        rightTopVelocity,
        indexer1Velocity,
        indexer2Velocity,
        hoodPosition);

    inputs.leftBottomVelocityRps = leftBottomVelocity.getValueAsDouble();
    inputs.leftTopVelocityRps = leftTopVelocity.getValueAsDouble();
    inputs.rightBottomVelocityRps = rightBottomVelocity.getValueAsDouble();
    inputs.rightTopVelocityRps = rightTopVelocity.getValueAsDouble();

    inputs.indexer1VelocityRps = indexer1Velocity.getValueAsDouble();
    inputs.indexer2VelocityRps = indexer2Velocity.getValueAsDouble();

    inputs.hoodAngleRads =
        (hoodPosition.getValueAsDouble() * 1.0 * 2 * Math.PI) - Math.toRadians(-32);
  }

  @Override
  public void setShooterVelocity(double velocityRps) {
    if (velocityRps == 0) {
      leftBottom.setControl(voltageRequest.withOutput(0));
    } else {
      leftBottom.setControl(shooterVelocityRequest.withVelocity(velocityRps));
    }
  }

  @Override
  public void setIndexerVelocity(double indexer1Rps, double indexer2Rps) {
    indexer1.setControl(indexerVelocityRequest.withVelocity(indexer1Rps));
    indexer2.setControl(indexerVelocityRequest.withVelocity(indexer2Rps));
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leftBottom.setControl(voltageRequest.withOutput(0));
    indexer1.setControl(voltageRequest.withOutput(0));
    indexer2.setControl(voltageRequest.withOutput(0));
    hoodMotor.setControl(voltageRequest.withOutput(0));
  }
}
