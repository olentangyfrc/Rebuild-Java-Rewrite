package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFXConfiguration leaderConfiguration;
  private TalonFXConfiguration followerConfiguration;
  private TalonFXConfiguration pivotConfiguration;

  private final int leaderMotorCanId = 28; // Replace with the actual leader motor port number
  private final int followerMotorCanId = 29; // Replace with the actual follower motor
  private final int pivotMotorCanId = 30; // Replace with the actual pivot motor port number

  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private TalonFX pivotMotor;

  public void init() {

    leaderConfiguration = new TalonFXConfiguration();
    leaderConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfiguration.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    leaderConfiguration.Slot0.kP = 0.21164 * 2;
    leaderConfiguration.Slot0.kI = 0.0;
    leaderConfiguration.Slot0.kD = 0.0;
    leaderConfiguration.Slot0.kS = 0.14158;
    leaderConfiguration.Slot0.kV = 0.12061;
    leaderConfiguration.Slot0.kA = 0.0038129;
    leaderMotor.getConfigurator().apply(leaderConfiguration, 0.25);

    followerConfiguration = new TalonFXConfiguration();
    followerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    followerMotor.getConfigurator().apply(followerConfiguration, 0.25);
    followerMotor.setControl(new Follower(leaderMotorCanId, MotorAlignmentValue.Opposed));

    pivotConfiguration = new TalonFXConfiguration();
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotMotor.getConfigurator().apply(pivotConfiguration, 0.25);
  }

  public Intake() {

    leaderMotor = new TalonFX(leaderMotorCanId, "can0");
    followerMotor = new TalonFX(followerMotorCanId, "can0");
    pivotMotor = new TalonFX(pivotMotorCanId, "can0");
  }

  public void setPosition(double position) {
    System.out.println("Intake pivot:" + position);
  }

  public void start() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(-30.0));
    System.out.println("Intake started");
  }

  public void stop() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    System.out.println("Intake stopped");
  }

  public void eject() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(30.0));
    System.out.println("Intake ejecting");
  }

  public void intakeIdle() {
    System.out.println("Intake idleing");
  }
}
