package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Interpolation;

public class Superstructure extends SubsystemBase {
  public enum SuperStructureState {
    IDLE,
    STASH,
    INTAKE,
    SHOOT,
    UNJAM,
    EJECT
  }

  private final Intake intake;
  private final Serializer serializer;
  private final Shooter shooter;
  private final frc.robot.subsystems.drive.Drive drive;
  private final Interpolation interpolation = new Interpolation();

  private SuperStructureState state = SuperStructureState.IDLE;
  private boolean emergencyStash = false;
  private boolean passing = false;

  public Superstructure(
      Intake intake,
      Serializer serializer,
      Shooter shooter,
      frc.robot.subsystems.drive.Drive drive) {
    this.intake = intake;
    this.serializer = serializer;
    this.shooter = shooter;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    if (state == SuperStructureState.SHOOT) {
      if (passing) {
        updatePassingParams(drive.getPassDistance());
      } else {
        setShootParams(drive.getDistanceToSpeaker());
      }
    }
    handleStateLogic();
  }

  private void updatePassingParams(double distance) {
    double angle = 0.39788 * Math.pow(distance, 2) - 4.2358 * distance + 41.14286;
    double rpm = 30.79247 * Math.pow(distance, 2) - 268.77109 * distance + 2185.71429;
    shooter.setShootParams(rpm, Math.toRadians(angle));
  }

  private void handleStateLogic() {
    switch (state) {
      case IDLE:
        if (shooter.atTargetSpeed()) { // Simplified cooldown logic from python
          shooter.setState(Shooter.ShooterState.COOLDOWN);
        } else {
          shooter.setState(Shooter.ShooterState.IDLE);
        }
        intake.setState(Intake.IntakeState.DEPLOYED);
        serializer.setState(Serializer.SerializerState.IDLE);
        break;

      case STASH:
        if (emergencyStash) {
          intake.setState(Intake.IntakeState.EMERGENCY_STASH);
        } else {
          intake.setState(Intake.IntakeState.STASHED);
        }
        serializer.setState(Serializer.SerializerState.IDLE);
        shooter.setState(Shooter.ShooterState.IDLE);
        break;

      case INTAKE:
        intake.setState(Intake.IntakeState.INTAKE);
        serializer.setState(Serializer.SerializerState.IDLE);
        shooter.setState(Shooter.ShooterState.IDLE);
        break;

      case SHOOT:
        if (shooter.atTargetSpeed()) {
          if (passing) {
            intake.setState(Intake.IntakeState.INTAKE_PASS);
            serializer.setState(Serializer.SerializerState.PASS_FORWARD);
          } else {
            if (intake.getState() != Intake.IntakeState.AGITATE
                && intake.getState() != Intake.IntakeState.STASHED) {
              serializer.setState(Serializer.SerializerState.FORWARD);
              intake.setState(Intake.IntakeState.AGITATE);
            }
          }
        } else {
          shooter.setState(Shooter.ShooterState.SPIN_UP);
          intake.setState(Intake.IntakeState.DEPLOYED);
          serializer.setState(Serializer.SerializerState.SLOW_REVERSE);
        }
        break;

      case UNJAM:
        serializer.setState(Serializer.SerializerState.REVERSE);
        shooter.setState(Shooter.ShooterState.EJECT);
        break;

      case EJECT:
        serializer.setState(Serializer.SerializerState.REVERSE);
        intake.setState(Intake.IntakeState.EJECT);
        break;
    }
  }

  public void setState(SuperStructureState newState) {
    this.state = newState;
  }

  public void setShootParams(double distance) {
    double[] values = interpolation.getInterpolatedValues(distance);
    if (values != null) {
      shooter.setShootParams(values[1], Math.toRadians(values[0]));
    }
  }

  public void setPassing(boolean passing) {
    this.passing = passing;
  }

  public void setEmergencyStash(boolean emergency) {
    this.emergencyStash = emergency;
  }
}
