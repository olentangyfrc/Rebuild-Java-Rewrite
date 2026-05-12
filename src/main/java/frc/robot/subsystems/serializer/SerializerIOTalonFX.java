package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SerializerIOTalonFX implements SerializerIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public SerializerIOTalonFX(int id, String canbus) {
    motor = new TalonFX(id, canbus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.33796;
    config.Slot0.kV = 0.12192;
    config.Slot0.kA = 0.0029284;
    config.Slot0.kP = 0.10721;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 9999;
    motor.getConfigurator().apply(config);

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    statorCurrent = motor.getStatorCurrent();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, appliedVolts, supplyCurrent, statorCurrent, temp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, appliedVolts, supplyCurrent, statorCurrent, temp);
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelcius = temp.getValueAsDouble();
  }

  @Override
  public void runVelocity(double velocityRps) {
    motor.setControl(velocityRequest.withVelocity(velocityRps));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0));
  }
}
