// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.util.misc.drive.AllianceFlipUtil;
import frc.robot.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  private static double slowMode = 1;   // slowMode speed is initialized as full speed
  public boolean foundTarget = false;   // used in limelight_ command

  // Create 4 MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    // Swap FL with FR to solve the X formation of the wheel when turn robot
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);
    
  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    // Swap FR with FL to solve the X formation of the wheel when turn robot
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
    // Swap BL with BR to solve the X formation of the wheel when turn robot
    DriveConstants.kBackRightDrivingCanId,
    DriveConstants.kBackRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

  private final MAXSwerveModule m_backRight = new MAXSwerveModule(
    // Swap BR with BL to solve the X formation of the wheel when turn robot
    DriveConstants.kBackLeftDrivingCanId,
    DriveConstants.kBackLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  // AdvantageKit 4.1.0\AdvantageKit_SparkSwerveTemplate\src\main\java\frc\robot\subsystems\drive\Drive.java
  // Put all modules into an array for easy manipulation
  private final MAXSwerveModule[] modules = 
    new MAXSwerveModule[] {
      m_frontLeft,    // FL
      m_frontRight,   // FR
      m_backLeft,     // BL
      m_backRight     // BR
    };

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private double resetYaw;

  // https://github.com/Talon-Inc/2024-Crescendo/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java
  // PID for teleop drift adjustment
  private final PIDController rotationPID;
  private double lastMovingYaw = 0.0;
  private boolean rotating = false;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java
  // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        getModulePositions()
    );

  // https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java
  // https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,        // kinemtaics
          m_gyro.getRotation2d(),                 // heading of the robot as a Rotation2d. Angle is continuous (e.g.: > 360 degrees)
          getModulePositions(),
          new Pose2d(),                           // initialPoseMeters
          // stateStdDevs - Standard deviations of the pose estimate [x,y positions in meters, thetaHeading in radians]. 
          // Increase these numbers to trust your state estimate less.
          VecBuilder.fill(0.05, 0.05, 
            Units.degreesToRadians(5)),
          // visionMeasurementStdDevs - Standard deviations of the vision pose measurement [x,y positions in meters, thetaHeading in radians].
          // Increase these numbers to trust the vision pose measurement less.
          VecBuilder.fill(0.5, 0.5, 
            Units.degreesToRadians(30)));

  // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Drive.java
  private SysIdRoutine sysId;
  //private SysIdRoutine turnRoutine;

  private Field2d field = new Field2d();
  
  // config for Pathplanner
  private RobotConfig config;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    //HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // https://github.com/SeanSun6814/FRC0ToAutonomous/blob/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot/subsystems/SwerveSubsystem.java
    // start a thread to calibrate Navx before resetting heading to 0
    new Thread(() -> {                 // create a separate thread so it doesn't interfere with other process
      try {
        Thread.sleep(1000);     // wait 1sec for gyro to calibrate 
        m_gyro.reset();                // before reset heading to 0
      } catch (Exception ex) {
            DriverStation.reportError("Error calibrating navX MXP:  " + ex.getMessage(), true);
      }
    }).start();
  
    // https://github.com/Talon-Inc/2024-Crescendo/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java
    lastMovingYaw = getYawAngle();
    rotationPID = new PIDController(DriveConstants.kteleopRotationP, DriveConstants.kteleopRotationI, DriveConstants.kteleopRotationD);
    rotationPID.enableContinuousInput(DriveConstants.kMinimumInput, DriveConstants.kMaximumInput);

    // Configure AutoBuilder for PathPlanner
    // Before configure the AutoBuilder for PP, make sure you:
    // 1. Import PathplannerLib
    // 2. Have the method getPose() : poseSupplier - a supplier for the robot's current pose
    // 3. Have the method setPose() : resetPose - a consumer for resetting the robot's pose
    //robotRelativeSpeedsSupplier - a supplier for the robot's current robot relative chassis speeds
    // 4. Have the method runVelocity()
    // 5. 

    // PID Constants used in AutoBuilder config
    PIDConstants translationPID = new PIDConstants(DriveConstants.kTranslationP, DriveConstants.kTranslationI, DriveConstants.kTranslationD);
    PIDConstants rotationPID = new PIDConstants(DriveConstants.kTurnAngleP, DriveConstants.kTurnAngleI, DriveConstants.kTurnAngleD);

    // Load the robot config from the shared settings file created by the GUI
    try {

      config = RobotConfig.fromGUISettings();
      //config = DriveConstants.ppConfig;

      // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Drive.java
      // https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java
      // Configure AutoBuilder for PathPlanner
      AutoBuilder.configure(
          // poseSupplier - a supplier for the robot's current pose
          this::getPose,
          // resetPose - a consumer for resetting the robot's pose (will be called if your auto has a starting pose)
          this::setPose,
          // robotRelativeSpeedsSupplier - a supplier for the robot's current robot relative chassis speeds
          // getRelativeChassisSpeeds: kinematics.toChassisSpeeds(getModuleStates()
          this::getSpeeds,
          /*
          () ->
              DriveConstants.kDriveKinematics.toChassisSpeeds(
                  getModuleStates()),
          */
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
          // output - Output function that accepts robot-relative ChassisSpeeds and feedforwards for each drive motor. 
          //   If using swerve, these feedforwards will be in FL, FR, BL, BR order. 
          // targetSpeeds: 
          // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Drive.java
          /*
          (speeds, feedforwards) ->
              runVelocity(
                  speeds),
          */
          this::driveRobotRelative,
          // PPHolonomicController is the built in path following controller for holonomic drive trains
          // controller - Path following controller that will be used to follow paths
          // Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( 
              translationPID, // Translation PID constants
              rotationPID // Rotation PID constants
              ),
          // robotConfig - The robot configuration
          config,
          // shouldFlipPath - Supplier that determines if paths should be flipped to the other side of the field. 
          //    This will maintain a global blue alliance origin.
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          // driveRequirements - the subsystem requirements for the robot's drive train
          this // Reference to this subsystem to set requirements
      );  // end AutoBuilder.configure

    } catch (Exception e) {
      //e.printStackTrace();
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", true);
    }

    // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Drive.java
    // Uncomment the next 2 lines if using AdvantageKit
    // Pathfinding.setPathfinder(new LocalADStarAK());    // if use AdvantageKit
    // Pathfinding.ensureInitialized();

    // https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java
    // this is sent as a list of poses to make the path
    // Setup custom logging to add the current path to a field2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    // display field on dashboard
    //SmartDashboard.putData("Field", field);

/*
        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
*/

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> {
                  for (MAXSwerveModule module : modules) {
                    module.runCharacterization(volts.in(Volts), 0);
                  }
                },
                null,
                this));
    //
/*    
    turnRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> {
                  for (MAXSwerveModule module : modules) {
                    module.runCharacterization(0, volts.in(Volts));
                  }
                },
                null,
                this));
*/
  } // end DriveSubsystem() constructor

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    /** Updates the poseEstimator. */
    updateOdometry();

    // update robot pose on the field
    field.setRobotPose(getPose());
    
    SmartDashboard.putNumber("Robot Heading (Deg)", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    SmartDashboard.putString("FL Position (m, rad)", m_frontLeft.getPosition().toString());
    SmartDashboard.putString("FR Position (m, rad)", m_frontRight.getPosition().toString());
    SmartDashboard.putString("BL Position (m, rad)", m_backLeft.getPosition().toString());
    SmartDashboard.putString("BR Position (m, rad)", m_backRight.getPosition().toString());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Drive.java
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    
    SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java
    //return m_odometry.getPoseMeters();
    SmartDashboard.putString("Pose (odometry)", m_odometry.getPoseMeters().toString());

    // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Drive.java
    SmartDashboard.putString("Pose (poseEstimator)", m_poseEstimator.getEstimatedPosition().toString());
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    //System.out.println("In setPose, pose = "+ pose);
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards ) {
    robotRelativeSpeeds.vyMetersPerSecond = -0.67 * robotRelativeSpeeds.vyMetersPerSecond; // risky sabotage from 714, delete if suspicious
    robotRelativeSpeeds.vxMetersPerSecond = 1.1 * robotRelativeSpeeds.vxMetersPerSecond; // another act of sabotage from 714, delete if unsure
   // robotRelativeSpeeds.omegaRadiansPerSecond = 1.5 * robotRelativeSpeeds.omegaRadiansPerSecond;

    
    robotRelativeSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(states);
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getYawAngle());
    // return getPose().getRotation();
  }
  
  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * slowMode;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * slowMode;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * slowMode;
    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void driveNotWorking(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // https://github.com/Talon-Inc/2024-Crescendo/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java
    if (m_currentRotation == 0) {
      if (rotating) {
        rotating = false;
        lastMovingYaw = getYawAngle();
      }
      m_currentRotation = rotationPID.calculate(getYawAngle(), lastMovingYaw);
    } else {
      rotating = true;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Toggle Slow Mode */
  public static void toggleSlowMode() {
    if (slowMode == 1) {
      slowMode = DriveConstants.kSlowModeConstant;
    } else {
      slowMode = 1;
    }
  }

  public void stop(){
    drive(0,0,0,false);
  }

// ------------------------- CHASSIS ----------------------------------------------------  
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public Command setXCommand() {
    // 'this' means Command
    return this.run(
      () -> {
        //new PrintCommand("setXCommand is running");
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      });
  }

  public Command setHCommand() {
    return this.run(
      () -> {
        //new PrintCommand("setHCommand is running");
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      });
  }

  public void setX() {
    //System.out.println ("setX() is running");
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setH() {
    //System.out.println ("setH() is running");
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and driveZ velocities) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public double getAverageEncoderDistance(){
    double averageEncoderDistance = 0.0;
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    states = getModulePositions();
    
    for (int i = 0; i < 4; i++) {
      averageEncoderDistance += Math.abs(states[i].distanceMeters);
    }
    return averageEncoderDistance / 4.0;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  // -------------------------- GYRO ------------------------------------------------------  
  public void resetYaw() {
    zeroAll();
    setPose(AllianceFlipUtil.apply(new Pose2d()));
  }

  // zero yaw, pitch and roll angles
  public void zeroAll() {
    zeroYawAngle();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //return  getYawAngle();
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Gets the current yaw angle.
   *
   * @return The angle in degrees limited to the range -180 to 180.
   */
  //@Override
  public double getYawAngle() {
    double angle = m_gyro.getAngle() - resetYaw;
    while (angle <= -180) angle += 360;
    while (angle > 180) angle -= 360;

    //return -angle;
    return angle * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  /**
   * Gets the current yaw angle velocity in deg / s
   *
   * @return The yaw angle rate in degrees.
   */
  public double getYawAngleVelocity() {
    return m_gyro.getRate();
  }

  /** Sets the current yaw angle to "0". */
  public void zeroYawAngle() {
    // navx.setAngleAdjustment(-navx.getAngle());
    resetYaw = m_gyro.getAngle();
  }

  /** Sets the current yaw angle to angle. */
  public void setYawAngle(double angle) {
    // navx.setAngleAdjustment(angle - navx.getAngle());
    resetYaw = m_gyro.getAngle() - angle;
  }

  //-------------------- ODOMETRY ----------------------------------------------------
  // https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
      // Update the poseEstimator (e.g.: gyroAngle, wheelPositions)
      m_poseEstimator.update(
          m_gyro.getRotation2d(),
          getModulePositions()
      );
  
      // if we want to update poseEstimator using limelight
      if (Constants.useVision){
        boolean useMegaTag2 = true; //LL2 uses MegaTag2. Set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
          try{
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
            {
              if(mt1.rawFiducials[0].ambiguity > .7)
              {
                doRejectUpdate = true;
              }
              if(mt1.rawFiducials[0].distToCamera > 3)
              {
                doRejectUpdate = true;
              }
            }
            if(mt1.tagCount == 0)
            {
              doRejectUpdate = true;
            }
      
            if(!doRejectUpdate)
            {
              m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
              m_poseEstimator.addVisionMeasurement(
                  mt1.pose,
                  mt1.timestampSeconds);
            }
          }
          catch (NullPointerException e) {}

        } // end if(useMegaTag2 == false)
        else if (useMegaTag2 == true)
        {
          // Sets robot orientation values used by MegaTag2 localization algorithm.
          LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

          // Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator 
          //  (addVisionMeasurement) in the WPILib Blue alliance coordinate system. 
          //  Make sure you are calling setRobotOrientation() before calling this method.
          try {
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
              doRejectUpdate = true;
            }

            if(mt2.tagCount == 0)
            {
              doRejectUpdate = true;
            }

            if(!doRejectUpdate)
            {
              m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
              m_poseEstimator.addVisionMeasurement(
                  mt2.pose,
                  mt2.timestampSeconds);
            }
          }
          catch (NullPointerException e){}
  
        } // end else if (useMegaTag2 == true)

      } // end if (Constants.useVision)

  } // end updateOdometry()

  // --------------- LIMELIGHT FUNCTIONS  ------------------------------------------------------------------
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  // https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-aim-and-range/src/main/java/frc/robot/Robot.java
  public double limelight_aim_proportional(){    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .002185;  //.00437; //.00875; //.0175; //.035;
    double targetingAngularVelocity = 0.0;

    System.out.print("In aim: ");

    if (LimelightHelpers.getTV("limelight") == false && !foundTarget){
      targetingAngularVelocity = 0.3; // We don't see the target, seek for the target by spinning in place CCW at a safe speed.
      System.out.println("");
    }
    else{
      foundTarget = true;
      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
      // your limelight3 feed, tx should return roughly 31 degrees.
      targetingAngularVelocity = LimelightHelpers.getTX("limelight");
      //System.out.print("TX = " + targetingAngularVelocity);
      targetingAngularVelocity *= kP;
      //System.out.print(", TX*kP = " + targetingAngularVelocity);

      // convert to radians per second for our drive method
      targetingAngularVelocity *= AutoConstants.kMaxAngularSpeed;
      //System.out.print(", TX*kP_Rad/sec = " + targetingAngularVelocity);

      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
      System.out.print(", targetingAngularVelocity = "+ targetingAngularVelocity);
      
      System.out.print(", TA="+LimelightHelpers.getTA("limelight"));
      System.out.println(", TV="+LimelightHelpers.getTV("limelight"));
    }
    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  // https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-aim-and-range/src/main/java/frc/robot/Robot.java
  public double limelight_range_proportional(){    
    double kP = .045; //.05; //.1;
    double targetingForwardSpeed = 0.0;

    System.out.print("In range: ");

    if (LimelightHelpers.getTV("limelight") == false){
      targetingForwardSpeed = 0.0;
      System.out.println("");
    }
    else{
      targetingForwardSpeed = LimelightHelpers.getTY("limelight");
      //System.out.print("TY = " + targetingForwardSpeed);
      targetingForwardSpeed *= kP;
      //System.out.print(", TY*kP = " + targetingForwardSpeed);

      targetingForwardSpeed *= AutoConstants.kAutoDriveSpeed;
      //System.out.print(", TY*kP*MaxSpeed = " + targetingForwardSpeed);
      targetingForwardSpeed *= -1.0;
      System.out.print(", targetingForwardSpeed = " + targetingForwardSpeed);

      System.out.print(", TA="+LimelightHelpers.getTA("limelight"));
      System.out.println(", TV="+LimelightHelpers.getTV("limelight"));
    }
    return targetingForwardSpeed*3;
  }

  // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-and-ranging

  public double[] limelight_aim_and_range(){

    double[] adjust = new double[2];
    double KpAim =  .0001;//.002138  //-0.1;
    double KpDistance = .03; //.025//-0.1;
    double min_aim_command = 0.05;
    double initialSpeed = .45;
    
    double heading_error = - LimelightHelpers.getTX("limelight");
    System.out.print("TX = "+ heading_error);
    double distance_error = - LimelightHelpers.getTY("limelight");//Experiment with using TA for TY
    System.out.print("TY = "+ distance_error);
    double angle_adjust = 0.0;

    if (heading_error > .3)
    {
        angle_adjust = KpAim * heading_error - min_aim_command;
        System.out.print(", TX > 1");
    }
    else if (heading_error < -.3)
    {
        angle_adjust = -KpAim*heading_error + min_aim_command;
        System.out.print(", TX < -1");
    }
    System.out.print(", steering_adjust = "+angle_adjust);
    adjust[0] = angle_adjust;

    double distance_adjust = KpDistance * distance_error;
    System.out.print(", distance_adjust = "+ distance_adjust);
    adjust[1] = initialSpeed-distance_adjust;

    return adjust;

  }

  // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
  public double limelight_estimate_distance(){
    // h2 = vertical distance from target to the ground
    //      LimelightConstants.goalHeightMeters
    // h1 = vertical distance from len of camera to the ground
    //      LimelightConstants.limelightLensHeightMeters
    // a1 = ty
    // a2 = camera mounting angle with recpect to horizontal axis
    //      LimelightConstants.limelightMountAngleDegrees
    // d = (h2-h1) / tan(a1+a2)

    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");                  // a2
    double angleToGoalDegrees = LimelightConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical; // (a1 + a2)
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);                                     // convert (a1+a2) from deg. to Radians

    //calculate distance d = (h2-h1) / tan(a1+a2)
    double distanceFromLimelightToGoalMeters = (LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters) / Math.tan(angleToGoalRadians);

    System.out.println("In limelight_estimate_distance");
    System.out.print("a1="+LimelightConstants.limelightMountAngleDegrees);
    System.out.print(", a2="+targetOffsetAngle_Vertical);
    System.out.print(", h1="+LimelightConstants.limelightLensHeightMeters);
    System.out.print(", h2="+LimelightConstants.goalHeightMeters);
    System.out.println(", d="+distanceFromLimelightToGoalMeters);

    return distanceFromLimelightToGoalMeters;
  }

  // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
  public double limelight_getting_in_range(double desired_distanceMeters){
    double KpDistance = -0.1;  // Proportional control constant for distance
    double current_distance = limelight_estimate_distance();  // see the 'Case Study: Estimating Distance' 

    double distance_error = (desired_distanceMeters - current_distance) * KpDistance;

    System.out.println("In limelight_getting_in_range, distance_error="+distance_error);

    return distance_error;
  }

  public static Command DriveAngleDistance(DriveSubsystem driveSubsystem, double distanceMeters, double driveAngleDeg){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
      driveSubsystem::resetEncoders,

      // onExecute: drive forward (if driveReversed = false) or reverse (if driveReversed = true) at an angle while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
/*
      () -> driveSubsystem.drive(
              -AutoConstants.kAutoDriveSpeed * ((distanceMeters<0) ? -1 : 1) * Math.cos(Units.degreesToRadians(driveAngleDeg)), // Forward
              AutoConstants.kAutoDriveSpeed * ((distanceMeters<0) ? -1 : 1) * Math.sin(Units.degreesToRadians(driveAngleDeg)), // Strafe
              0,                                                                                                      // Rotation
              false), 
*/                                                                                      // fieldRelative ?
      () -> {driveSubsystem.drive(
                -AutoConstants.kAutoDriveSpeed * ((distanceMeters<0) ? -1 : 1) * Math.cos(Units.degreesToRadians(driveAngleDeg)), // Forward
                AutoConstants.kAutoDriveSpeed * ((distanceMeters<0) ? -1 : 1) * Math.sin(Units.degreesToRadians(driveAngleDeg)), // Strafe
                0,                                                                                                      // Rotation
                false);
                driveSubsystem.getAverageEncoderDistance();                                                                                      // fieldRelative ?
            },
      //()-> driveSubsystem.omniDirectionStrafe(AutoConstants.kAutoOmniDirectionSpeed * (driveReversed ? -1 : 1), driveAngleDeg),

      // onEnd: stop driving at the end of command
      //interrupt -> driveSubsystem.setX(), 
      interrupt -> driveSubsystem.stop(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      () -> Math.abs(driveSubsystem.getAverageEncoderDistance()) >= Math.abs(distanceMeters),
      
      // require the drive subsystem
      driveSubsystem);
  }
  
  /*public static Command RightReefAlign(DriveSubsystem subsystem, double distance, double angle) {
    return this.startEnd(
      () -> this.DriveAngleDistance(subsystem, distance, angle), () -> this.DriveAngleDistance(subsystem, distance, angle));
    
  }*/
} // end class DriveSystem
