package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFXConfiguration leaderConfiguration;
  private TalonFXConfiguration followerConfiguration;
  private TalonFXConfiguration pivotConfiguration;

  private final int leaderMotorCanId = 28; // Replace with the actual leader motor port number
  private final int followerMotorCanId = 29; // Replace with the actual follower motor
  private final int pivotMotorCanId = 30;
  private final int pivotEncoderCanId = 51; // Replace with the actual pivot motor port number

  private final double pivotGearRatio = 1 / 2.75;
  private final double encoderOffset = 14; // Replace with the actual encoder offset

  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private TalonFX pivotMotor;
  private CANcoder pivotEncoder;

  private PIDController pivotPIDController;
  private ArmFeedforward pivotFFWController;
  private double pivotTargetAngle;

  private double angle;

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
    pivotConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    pivotMotor.getConfigurator().apply(pivotConfiguration, 0.25);
  }

  public Intake() {

    leaderMotor = new TalonFX(leaderMotorCanId, "can0");
    followerMotor = new TalonFX(followerMotorCanId, "can0");
    pivotMotor = new TalonFX(pivotMotorCanId, "can0");
    pivotEncoder = new CANcoder(pivotEncoderCanId, "can0");

    pivotPIDController = new PIDController(4, 0, 0.1);
    pivotFFWController = new ArmFeedforward(0.1, 0.42, 0);
    pivotPIDController.setTolerance(Math.toRadians(5));
    // pivotPIDController.setIZone(Math.toRadians(0));
    // pivotPIDController.setSetpoint(Math.toRadians(0));
    pivotPIDController.reset();
  }

  public double getPivotAngle() {
    double angle =
        (Math.toRadians(pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360)
                * pivotGearRatio
            - Math.toRadians(encoderOffset));

    if (angle < Math.toRadians(-15)) angle += Math.toRadians(159.8);
    return angle;
  }

  public void setPivotSetPoint(double pivotSetPoint) {
    pivotTargetAngle = MathUtil.clamp(pivotSetPoint, Math.toRadians(0.5), Math.toRadians(125));
  }

  static double lastAngle = 0;

  public void periodic() {
    pivotMotor.setControl(
        new com.ctre.phoenix6.controls.VoltageOut(
            pivotPIDController.calculate(getPivotAngle(), pivotTargetAngle)
                + pivotFFWController.calculate(getPivotAngle(), 0)));

    // Logger.getLogger("Pivot Angle: " + Math.toDegrees(getPivotAngle()));
    // System.out.println(Math.toDegrees(getPivotAngle()));

    // if (getPivotAngle() - lastAngle > Math.toRadians(1)) {
    //   System.out.println("Pivot Angle: " + Math.toDegrees(getPivotAngle()));
    //   lastAngle = getPivotAngle();
    // }
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
