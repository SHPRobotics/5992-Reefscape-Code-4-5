// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // if we want to use limelight to correct robot pose
  public static final boolean useVision = true;

  public static final class CoralSubsystemConstants {
    public static final int kShooterMotorCanId = 9;
    //public static final int kIntakeLeftMotorCanId = 10;
    public static final int kElevatorMotorCanId = 13;
    public static final int kEntryCoralMotorCanId = 14;
    

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;//level 1 will havee limit switch // this is the min height 
      public static final int kLevel2 = 37;//manual height testing required
      public static final int kLevel3 = 91;//level 3 will have limit switch  // this is the max height
      public static final int kLevel4 = 150;

      public static final double kElevatorTopPosition = 200.0;     // The max position the elevator can go
      public static final double kElevatorBotomPosition = 0.0;    // The Minimum position the elevator can go
    }

    
    public static final class IntakeSetpoints {
      public static final double kForward = 0.3;
      public static final double kReverse = -0.5;
    }
  }

  public static final class AlgaeSubsystemConstants {
    public static final int kIntakeMotorCanId = 11;
    public static final int kPivotMotorCanId = 12;

    public static final double kArmIntakeKp = 0.025;
    public static final double kArmSpeed = 0.8;
    
    public static final class ArmSetpoints {
      public static final double kMaxPosition = -40;
      public static final double kMinPosition = 0;
      public static final double kStow = 0;//-18.5;//18.5;
      public static final double kHold = 11.5;
      public static final double kDown = 0;

      public static final double kLevelGround = 0;
      public static final double kLevel2 = 0;
      public static final double kLevel3 = 0;
      public static final double kLevelOnCoral = 0;
      public static final double kOuttake = 0;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
      public static final double kHold = 0.25;
      public static final double kStop = 0.0;
    }
  }

  public static final class AlgaeConstants{
    //public static final int kHighAlgaeCANId = 11;
    //public static final int kLowAlgaeCANId = 12;

    //public static final IdleMode kAlgaeMotorIdleMode = IdleMode.kBrake;
    //public static final int kAlgaeMotorCurrentLimit = 40; // amps
      
    //public static final double kAlgaeSpeedOut = 1.0;
    //public static final double kAlgaeSpeedIn = 0.3;
  }

  public static final class CoralConstants{
    //public static final int kLeftCoralCANId = 9;
    //public static final int kRightCoralCANId = 10;

    public static final IdleMode kCoralMotorIdleMode = IdleMode.kBrake;
    public static final int kCoralMotorCurrentLimit = 40; // amps
    
    public static final double kCoralSpeedOut = 1.5;
    public static final double kCoralSpeedIn = 0.3;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kSlowModeConstant = 0.5;
    public static final int kDriveCurrentLimit = 50; // amps
    public static final int kTurnCurrentLimit = 20; // amps

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Teleop drift adjustment PID constants
    public static final double kteleopRotationP = 0.0004;
    public static final double kteleopRotationI = 0.0;
    public static final double kteleopRotationD = 0.0;
    public static final double kMinimumInput = -180.0;
    public static final double kMaximumInput = 180.0;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    public static final double driveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };
    
    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;    // -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;   // 0;
    public static final double kBackLeftChassisAngularOffset = 0;     // Math.PI;
    public static final double kBackRightChassisAngularOffset = 0;    //Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kBackLeftDrivingCanId = 5;
    public static final int kBackRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kBackLeftTurningCanId = 6;
    public static final int kBackRightTurningCanId = 8;

    public static final boolean kGyroReversed = true;

    // PATHPLANNER
    public static final double kTranslationP = 1.0; //10.0;  //3.50;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;
  
    public static final double kTurnAngleP = 0.04;  //7.0;   //0.9;
    public static final double kTurnAngleI = 0;
    public static final double kTurnAngleD = 0;

    public static final double kTurnAngleTolerance = 0.05;
    public static final double kTurnAngleRateTolerance = 0.02;

    // Pathplanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;

    // from AK4.1.0\RobotContainer.java
    // robotConfig for Pathplanner
    public static final RobotConfig ppConfig =
      new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
          ModuleConstants.kWheelDiameterMeters / 2.0,
          DriveConstants.kMaxSpeedMetersPerSecond,
          wheelCOF, 
          DCMotor.getNEO(1), 
          DriveConstants.kDriveCurrentLimit, 
          1),
        moduleTranslations
      );
    
  } // end DriveConstants

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 
      // 0.0762;  // MAXSwerve Module diameter 3" or 0.0752 m
      0.1016;     // SDS Mk4i Module diameter 4" or 0.1016 m
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        // (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);   // MAXSwerve Module Medium speed = 5.08
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);      // SDS Mk4i Module L2 ratio = 6.75 (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kAutoDriveDistanceMeters = 3.0;
    public static final double kAutoDriveSpeed = 0.3;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }

  public static final class LimelightConstants{
    public static final double limelightMountAngleDegrees = 0.0;
    // distance from the center of the Limelight lens to the floor
    public static final double limelightLensHeightMeters = Units.inchesToMeters(28.0); 
    // distance from the target to the floor
    public static final double goalHeightMeters = Units.inchesToMeters(25.0); 

  }
  
  public static final class LedConstants{
    public static final int kLedChannel = 5;

    public static final int PWM_LED_PORT = 9;
    public static final int LED_LENGTH = 60;
  }
  public static final class Vision {
    public static final String FRONT_CAMERA_NAME = "limelight-front";

  public static final double CAMERA_LEFT_OFFSET = Units.inchesToMeters(-8.5); //measurement from camera to left side of reef pipe
  public static final double CAMERA_RIGHT_OFFSET = Units.inchesToMeters(4.5); //measurement from camera to right side of reef pipe

  //public static final double CAMERA_CENTER_OFFSET = Units.inchesToMeters(-0.75);
  }
}
  



