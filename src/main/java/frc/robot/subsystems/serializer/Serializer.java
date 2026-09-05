package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Serializer extends SubsystemBase {
  private TalonFXConfiguration serializerConfig;

  private final int motorcan = 32; // Replace with the actual motor port number
  private TalonFX serializerMotor;

  public void init() {
    serializerConfig = new TalonFXConfiguration();
    serializerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    serializerConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    serializerConfig.Slot0.kP = 0.10721;
    serializerConfig.Slot0.kI = 0.0;
    serializerConfig.Slot0.kD = 0.0;
    serializerConfig.Slot0.kS = 0.33796;
    serializerConfig.Slot0.kV = 0.12192;
    serializerConfig.Slot0.kA = 0.0029284;
    serializerMotor.getConfigurator().apply(serializerConfig, 0.25);
  }

  public Serializer() {
    serializerMotor = new TalonFX(motorcan, "can0");
  }

  public void start() {
    serializerMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(15.0));
    System.out.println("Serializer started");
  }

  public void stop() {
    serializerMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    System.out.println("Serializer stopped");
  }

  public void reverse() {
    serializerMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(-15.0));
    System.out.println("Serializer reversing");
  }
}
