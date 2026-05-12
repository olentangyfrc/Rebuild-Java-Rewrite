package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {
  public enum SerializerState {
    IDLE,
    FORWARD,
    REVERSE,
    SLOW_REVERSE,
    PASS_FORWARD
  }

  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();

  private SerializerState state = SerializerState.IDLE;
  private double targetRps = 0.0;

  public Serializer(SerializerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    handleStateLogic();
    Logger.processInputs("Serializer", inputs);

    io.setRollerVelocity(targetRps);
    inputs.state = state.name();
  }

  private void handleStateLogic() {
    switch (state) {
      case IDLE:
        targetRps = 0;
        break;
      case FORWARD:
        targetRps = 85;
        break;
      case REVERSE:
        targetRps = -85;
        break;
      case SLOW_REVERSE:
        targetRps = -20;
        break;
      case PASS_FORWARD:
        targetRps = 40;
        break;
    }
  }

  public void setState(SerializerState newState) {
    this.state = newState;
  }

  public Command idleCommand() {
    return runOnce(() -> state = SerializerState.IDLE).withName("SerializerIdle");
  }

  public Command forwardCommand() {
    return runOnce(() -> state = SerializerState.FORWARD).withName("SerializerForward");
  }

  public Command reverseCommand() {
    return runOnce(() -> state = SerializerState.REVERSE).withName("SerializerReverse");
  }

  public Command slowReverseCommand() {
    return runOnce(() -> state = SerializerState.SLOW_REVERSE).withName("SerializerSlowReverse");
  }

  public Command passForwardCommand() {
    return runOnce(() -> state = SerializerState.PASS_FORWARD).withName("SerializerPassForward");
  }
}
