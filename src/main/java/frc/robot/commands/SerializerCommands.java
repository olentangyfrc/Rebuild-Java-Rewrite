package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.serializer.Serializer;

public class SerializerCommands {

  Serializer serializer;

  public static Command startSerializer(Serializer serializer) {
    return Commands.run(
        () -> {
          serializer.start();
          System.out.println("Serializer command started");
        },
        serializer);
  }

  public static Command stopSerializer(Serializer serializer) {
    return Commands.run(
        () -> {
          serializer.stop();
          System.out.println("Serializer command stopped");
        },
        serializer);
  }

  public static Command reverseSerializer(Serializer serializer) {
    return Commands.run(
        () -> {
          serializer.reverse();
        },
        serializer);
  }
}
