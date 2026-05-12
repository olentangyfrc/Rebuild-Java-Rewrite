package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {
  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();

  public Serializer(SerializerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Serializer", inputs);
  }

  public Command runForward() {
    return run(() -> io.runVelocity(85.0)).withName("SerializerForward");
  }

  public Command runReverse() {
    return run(() -> io.runVelocity(-85.0)).withName("SerializerReverse");
  }

  public Command runSlowReverse() {
    return run(() -> io.runVelocity(-20.0)).withName("SerializerSlowReverse");
  }

  public Command runPassForward() {
    return run(() -> io.runVelocity(40.0)).withName("SerializerPassForward");
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("SerializerStop");
  }
}
