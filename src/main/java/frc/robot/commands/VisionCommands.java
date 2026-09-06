package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;

public class VisionCommands {

  Vision vision;

  public static Command setupVision(Vision vision) {
    return Commands.run(
        () -> {
          vision.setup();
        },
        vision);
  }
}
