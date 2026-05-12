package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperStructureState;

public class SuperstructureCommands {
  private final Superstructure superstructure;
  private final frc.robot.subsystems.drive.Drive drive;

  public SuperstructureCommands(
      Superstructure superstructure, frc.robot.subsystems.drive.Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
  }

  public Command stashCommand() {
    return Commands.runOnce(
            () -> {
              superstructure.setEmergencyStash(false);
              superstructure.setState(SuperStructureState.STASH);
            },
            superstructure)
        .withName("SuperstructureStash");
  }

  public Command idleCommand() {
    return Commands.runOnce(() -> superstructure.setState(SuperStructureState.IDLE), superstructure)
        .withName("SuperstructureIdle");
  }

  public Command intakeFuelCommand() {
    return Commands.runOnce(
            () -> superstructure.setState(SuperStructureState.INTAKE), superstructure)
        .withName("SuperstructureIntake");
  }

  public Command shootFuelCommand(double distanceMeters) {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  superstructure.setPassing(false);
                  superstructure.setShootParams(distanceMeters);
                  superstructure.setState(SuperStructureState.SHOOT);
                },
                superstructure),
            drive.goToRotation(drive.getAngleToSpeaker()))
        .withName("SuperstructureShoot");
  }

  public Command passFuelCommand() {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  superstructure.setPassing(true);
                  superstructure.setState(SuperStructureState.SHOOT);
                },
                superstructure),
            drive.goToRotation(drive.getPassAngle()))
        .withName("SuperstructurePass");
  }

  public Command unjamCommand() {
    return Commands.runOnce(
            () -> superstructure.setState(SuperStructureState.UNJAM), superstructure)
        .withName("SuperstructureUnjam");
  }

  public Command ejectFuelCommand() {
    return Commands.runOnce(
            () -> superstructure.setState(SuperStructureState.EJECT), superstructure)
        .withName("SuperstructureEject");
  }
}
