// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static double slowMode = 1;
  private static boolean shootSide = false;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static PIDController angleController =
      new PIDController(DriveConstants.kTurnAngleP, DriveConstants.kTurnAngleI, DriveConstants.kTurnAngleD);
//  private static LoggedNetworkBoolean shootSide =
//      new LoggedNetworkBoolean("/SmartDashboard/ShootSide", false);

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * slowMode, DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped = getIsFlipped();
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.kMaxSpeedMetersPerSecond,
                  linearVelocity.getY() * DriveConstants.kMaxSpeedMetersPerSecond,
                  omega * DriveConstants.kMaxAngularSpeed,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDriveRobotRelative(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * slowMode, DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to robot relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.kMaxSpeedMetersPerSecond,
                  linearVelocity.getY() * DriveConstants.kMaxSpeedMetersPerSecond,
                  omega * DriveConstants.kMaxAngularSpeed,
                  new Rotation2d()));
        },
        drive);
  }

  private static boolean getIsFlipped() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  /** Drive robot while pointing at a specific point on the field. */
  public static Command joystickAnglePoint(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetDirection) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);

          double omega =
              angleController.calculate(
                  MathUtil.angleModulus(drive.getRotation().getRadians()),
                  targetDirection.get().getRadians());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to robot relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.kMaxSpeedMetersPerSecond,
                  linearVelocity.getY() * DriveConstants.kMaxSpeedMetersPerSecond,
                  omega * DriveConstants.kMaxAngularSpeed,
                  getIsFlipped()
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /** Toggle Slow Mode */
  public static void toggleSlowMode() {
    if (slowMode == 1) {
      slowMode = DriveConstants.kSlowModeConstant;
    } else {
      slowMode = 1;
    }
  }

  public static boolean getPivotSideAngle() {
    //if (shootSide.get()) {
    if (shootSide){
      return false;
    }
    return true;
  }


  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(DriveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }


  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

}
