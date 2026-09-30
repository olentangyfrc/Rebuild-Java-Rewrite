package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ProfileConstants;

public class Intake extends SubsystemBase {
  public static final ProfileConstants pivot_profile_constants = new ProfileConstants(500, 250);

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

  private ProfiledPIDController pivotPIDController;
  private ArmFeedforward pivotFFWController;
  private double pivotTargetAngle;

  private final Timer agitationTimer = new Timer();
  private boolean isAgitating = false;

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

    pivotPIDController =
        new ProfiledPIDController(
            4,
            0,
            0.1,
            new TrapezoidProfile.Constraints(
                pivot_profile_constants.maxVelocity(), pivot_profile_constants.maxAcceleration()));
    pivotFFWController = new ArmFeedforward(0.1, 0.42, 0);
    pivotPIDController.setTolerance(Math.toRadians(5));
    // pivotPIDController.setIZone(Math.toRadians(0));
    // pivotPIDController.setSetpoint(Math.toRadians(0));
    pivotPIDController.reset(getPivotAngle());
  }

  public double getPivotAngle() {
    double angle =
        (Math.toRadians(pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360)
                * pivotGearRatio
            - Math.toRadians(encoderOffset));

    if (angle < Math.toRadians(-15)) angle += Math.toRadians(159.8);
    return angle;
  }
  // starts intake to Agitation (U will lost setPoint Control During this)
  public void setIntakeAgitation(boolean ON) {
    if (ON) {
      if (!isAgitating) {
        agitationTimer.restart();
        isAgitating = true;
      }
      intakeIdle();

      double time = agitationTimer.get();

      if (time < 0.65) {
        setPivotSetPoint(Math.toRadians(25));
      } else if (time < 1.0) {
        setPivotSetPoint(Math.toRadians(60));
      } else if (time < 1.5) {
        setPivotSetPoint(Math.toRadians(125));
      } else {
        agitationTimer.restart(); // Loop the sequence back to the beginning
      }
    } else {
      if (isAgitating) {
        agitationTimer.stop();
        agitationTimer.reset();
        isAgitating = false;
        setPivotSetPoint(0);
        intakeIdle();
      }
    }
  }

  private double staticDownVoltage = 0.0;

  public void setStaticDownVoltage(double voltage) {
    this.staticDownVoltage = voltage;
  }

  public void driveIntakeDown(double voltage) {
    setPivotSetPoint(Math.toRadians(0.5));
    start();
    setStaticDownVoltage(voltage);
  }

  public void setPivotSetPoint(double pivotSetPoint) {
    pivotTargetAngle = MathUtil.clamp(pivotSetPoint, Math.toRadians(0.5), Math.toRadians(125));
  }

  public void resetIntake() {
    staticDownVoltage = 0.0;
    setPivotSetPoint(0);
    intakeIdle();
  }

  public void startAgitationIntake() {
    setIntakeAgitation(true);
  }

  public void stopAgitationIntake() {
    setIntakeAgitation(false);
  }

  public void startagitationIntake() {
    startAgitationIntake();
  }

  public void stopagitationIntake() {
    stopAgitationIntake();
  }

  public void periodic() {
    double pidOutput = pivotPIDController.calculate(getPivotAngle(), pivotTargetAngle);
    double ffOutput = pivotFFWController.calculate(getPivotAngle(), 0);
    double totalVoltage = pidOutput + ffOutput + staticDownVoltage;
    pivotMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(totalVoltage));
    if(getPivotAngle() > 35){
      leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(0.0));
    }
  }

  // tell velocity in RPS
  public void setIntakeRollersCustom(double velocity) {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(velocity));
  }

  public void start() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(-100.0));
  }

  public void stop() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }

  public void eject() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(100.0));
  }

  public void intakeIdle() {
    leaderMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(-15.0));
  }

  public void stash() {
    stopAgitationIntake();
    setPivotSetPoint(Math.toRadians(125));
    intakeIdle();
  }

  public void resetPivotEncoder() {
    pivotEncoder.setPosition(0);
  }

  public void adjustPivotAngle(double deltaRad) {
    setPivotSetPoint(pivotTargetAngle + deltaRad);
  }
}
