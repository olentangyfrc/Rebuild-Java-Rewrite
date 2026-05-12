package frc.robot.subsystems.intake;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivotMotor;
  private final CANcoder pivotEncoder;

  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Current> rollerStatorCurrent;

  private final VoltageOut pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final MotionMagicVelocityVoltage rollerVelocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut rollerStopRequest = new VoltageOut(0).withEnableFOC(true);

  public IntakeIOTalonFX() {
    pivotMotor = new TalonFX(30, "can0");
    pivotEncoder = new CANcoder(51, "can0");

    leftRollerMotor = new TalonFX(29, "can0");
    rightRollerMotor = new TalonFX(28, "can0");

    // Roller config
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Slot0.kS = 0.14158;
    rollerConfig.Slot0.kV = 0.12061;
    rollerConfig.Slot0.kA = 0.0038129;
    rollerConfig.Slot0.kP = 0.42328;
    rollerConfig.MotionMagic.MotionMagicAcceleration = 9999;

    leftRollerMotor.getConfigurator().apply(rollerConfig);
    rightRollerMotor.getConfigurator().apply(rollerConfig);
    rightRollerMotor.setControl(new Follower(29, MotorAlignmentValue.Opposed));

    // Pivot config
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotor.getConfigurator().apply(pivotConfig);

    pivotPosition = pivotEncoder.getAbsolutePosition();
    pivotVelocity = pivotEncoder.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    pivotStatorCurrent = pivotMotor.getStatorCurrent();

    rollerVelocity = leftRollerMotor.getVelocity();
    rollerAppliedVolts = leftRollerMotor.getMotorVoltage();
    rollerSupplyCurrent = leftRollerMotor.getSupplyCurrent();
    rollerStatorCurrent = leftRollerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrent,
        rollerStatorCurrent);

    pivotMotor.optimizeBusUtilization();
    leftRollerMotor.optimizeBusUtilization();
    rightRollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrent,
        rollerStatorCurrent);

    double gearRatio = 1.0 / 2.25;
    double encoderOffset = Units.degreesToRadians(47 - 9 - 14 + 1.1);

    inputs.pivotAngleRads =
        (pivotPosition.getValueAsDouble() * gearRatio * 2 * Math.PI) - encoderOffset;
    if (inputs.pivotAngleRads < Math.toRadians(-15)) {
      inputs.pivotAngleRads += Math.toRadians(159.8);
    }

    inputs.pivotVelocityRadsPerSec = pivotVelocity.getValueAsDouble() * gearRatio * 2 * Math.PI;
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();

    inputs.rollerVelocityRps = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
    inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double velocityRps) {
    if (velocityRps == 0.0) {
      leftRollerMotor.setControl(rollerStopRequest.withOutput(0));
    } else {
      leftRollerMotor.setControl(rollerVelocityRequest.withVelocity(velocityRps));
    }
  }

  @Override
  public void stop() {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(0));
    leftRollerMotor.setControl(rollerStopRequest.withOutput(0));
  }
}
