// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class Configs {

  public static final class MAXSwerveModule {
    // Instantiate SparkMaxConfig objects for driving motor and turning motor
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

      // All configs must be inside the static block
      static {

        // Use module constants to calculate conversion factors and feed forward gain.
        double drivingFactor = ModuleConstants.kWheelRadiusMeters * 2 * Math.PI / ModuleConstants.kDrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

        // Configure driving motor
        drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.kDriveCurrentLimit);

        // Configure driving encoder
        drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        
        // Configure driving closed loop controller
        drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
//          .pid(0.04, 0, 0)
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

        // Configure turning motor
        turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.kTurnCurrentLimit);

        // Configure turning absolute encoder
        turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second

        // Configure turning  closed loop controller
        turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);

      } // end of static

  } // end of MAXSwerveModule

  public static final class CoralSubsystem {
    // Coral subsystem has 1 elevator motor and 2 shooter (intake) motors
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    //public static final SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    public static final SparkMaxConfig entryIntakeConfig = new SparkMaxConfig();

    static {

      // Configure basic settings of the intake left motor
      /*intakeLeftConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);*/

      // Configure basic settings of the intake right motor
      shooterConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);
      // Configure basic settings of the intake right motor
      entryIntakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

      // Configure basic settings of the elevator motor
      elevatorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      /*
      elevatorConfig.limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyClosed)

          .forwardLimitSwitchEnabled(true)
          .forwardLimitSwitchType(Type.kNormallyClosed);
      
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1);
          /*
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);
          */
    } // end of static

  } // end of CoralSubsystem

  public static final class AlgaeSubsystem {
    // Algae subsystem has 1 arm motor and 1 intake motor
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the arm motor
      armConfig.smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.045)
        .outputRange(-0.5, 0.5);

      // Configure basic settings of the intake motor
      intakeConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

    } // end of static
  } // end of AlgaeSubsystem

} // end of Configs
