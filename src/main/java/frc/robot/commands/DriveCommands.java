// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 9.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 40.0;
  private static final double ANGLE_MAX_ACCELERATION = 60.0;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Linear magnitude mapping (un-squared for full linear stick response across range)
    // linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Field-relative drive command using joysticks for linear control and PID targeting to snap the
   * drivetrain heading to a fixed target angle (e.g. 0°, 90°, 180°).
   */
  public static Command snapToAngle(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Rotation2d targetAngle) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> targetAngle);
  }

  /**
   * Overloaded snapToAngle command without joystick translation input (rotates in place to target
   * angle).
   */
  public static Command snapToAngle(Drive drive, Rotation2d targetAngle) {
    return snapToAngle(drive, () -> 0.0, () -> 0.0, targetAngle);
  }

  /**
   * Field relative drive command using joysticks for linear control and PID targeting to
   * continuously point/align the drivetrain heading toward the Hub.
   */
  public static Command pointToHub(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, drive::getRotationToHub);
  }

  /**
   * Snake drive command: Field-relative drive using joysticks for linear movement, where the front
   * of the robot continuously points toward the direction of momentum (linear velocity).
   */
  public static Command snakeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    var lastTargetHeading = new Rotation2d[] {drive.getRotation()};

    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          if (linearVelocity.getNorm() > DEADBAND) {
            boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            Rotation2d movementAngle = new Rotation2d(linearVelocity.getX(), linearVelocity.getY());
            if (isFlipped) {
              movementAngle = movementAngle.plus(new Rotation2d(Math.PI));
            }
            lastTargetHeading[0] = movementAngle;
          }
          return lastTargetHeading[0];
        });
  }

  /**
   * Field-relative drive command using joysticks for linear control and PID targeting to
   * continuously point/align the drivetrain heading toward the velocity-compensated virtual Hub.
   * Works both on the move and while stationary.
   */
  public static Command shootForHub(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, drive::getShootForHubRotation);
  }

  /**
   * Overloaded shootForHub command without joystick translation input (points at virtual Hub in
   * place).
   */
  public static Command shootForHub(Drive drive) {
    return shootForHub(drive, () -> 0.0, () -> 0.0);
  }

  /**
   * Field-relative drive command using joysticks to point at the virtual Hub.
   *
   * @deprecated Use {@link #shootForHub(Drive, DoubleSupplier, DoubleSupplier)} instead.
   */
  @Deprecated
  public static Command shootOnTheMove(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return shootForHub(drive, xSupplier, ySupplier);
  }
public static Command passOnTheMove(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              double x = xSupplier.getAsDouble();
              double y = ySupplier.getAsDouble();
              Translation2d linearVelocity = getLinearVelocityFromJoysticks(x, y);

              if (linearVelocity.getNorm() <= 0.0) {
                drive.stopWithX();
              } else {
                double omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(),
                        drive.getPassRotation().getRadians());

                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              }
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Field-relative drive command using PID controllers for X position, Y position, and Rotation to
   * drive the robot to a target Pose2d.
   */
  public static Command driveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    @SuppressWarnings("resource")
    PIDController xController = new PIDController(5.0, 0.0, 0.0);
    @SuppressWarnings("resource")
    PIDController yController = new PIDController(5.0, 0.0, 0.0);
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              Pose2d targetPose = poseSupplier.get();

              double vx = xController.calculate(currentPose.getX(), targetPose.getX());
              double vy = yController.calculate(currentPose.getY(), targetPose.getY());
              double omega =
                  angleController.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians());

              Translation2d linearVelocity = new Translation2d(vx, vy);
              double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
              if (linearVelocity.getNorm() > maxSpeed) {
                linearVelocity = linearVelocity.times(maxSpeed / linearVelocity.getNorm());
              }

              ChassisSpeeds speeds =
                  new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xController.reset();
              yController.reset();
              angleController.reset(drive.getRotation().getRadians());
            });
  }

  /**
   * Field-relative drive command using PID controllers to drive the robot to a static target
   * Pose2d.
   */
  public static Command driveToPose(Drive drive, Pose2d targetPose) {
    return driveToPose(drive, () -> targetPose);
  }

  /** Stops the drive and locks swerve modules in an X arrangement to resist movement. */
  public static Command lock(Drive drive) {
    return Commands.run(drive::stopWithX, drive);
  }

  /**
   * Follows a Choreo trajectory loaded by name using PathPlanner's AutoBuilder.
   *
   * @param drive the drive subsystem
   * @param trajectoryName the name of the Choreo trajectory file (without extension)
   * @return a command that follows the Choreo trajectory
   */
  public static Command followChoreoPath(Drive drive, String trajectoryName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajectoryName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load Choreo trajectory: " + trajectoryName + " - " + e.getMessage(),
          e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Follows a split Choreo trajectory loaded by name and split index using PathPlanner's
   * AutoBuilder.
   *
   * @param drive the drive subsystem
   * @param trajectoryName the name of the Choreo trajectory file (without extension)
   * @param splitIndex the index of the split section of the trajectory
   * @return a command that follows the split Choreo trajectory
   */
  public static Command followChoreoPath(Drive drive, String trajectoryName, int splitIndex) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajectoryName, splitIndex);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load Choreo trajectory: "
              + trajectoryName
              + " (split "
              + splitIndex
              + ") - "
              + e.getMessage(),
          e.getStackTrace());
      return Commands.none();
    }
  }

  /** Resets the robot pose to (0, 0, 0°). */
  public static Command resetPoseToZero(Drive drive) {
    return Commands.runOnce(() -> drive.setPose(Pose2d.kZero), drive).ignoringDisable(true);
  }
}
