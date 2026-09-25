package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.serializer.Serializer;

public class SerializerCommands {
  private SerializerCommands() {}

  public static Command startSerializer(Serializer serializer) {
    return Commands.run(() -> serializer.start(), serializer);
  }

  public static Command stopSerializer(Serializer serializer) {
    return Commands.run(() -> serializer.stop(), serializer);
  }

  public static Command reverseSerializer(Serializer serializer) {
    return Commands.run(() -> serializer.reverse(), serializer);
  }
}
