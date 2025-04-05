// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
// https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
public class MAXSwerveModule {
    // Motors
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;
    
    // drive Relative encoder
    private final RelativeEncoder m_drivingEncoder;
    // turn Absolute encoder
    private final AbsoluteEncoder m_turningEncoder;

    // Closed Loop controllers
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    // Variables
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
    /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    // Instantiate motors
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    // Instantiate encoders
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    // Instantiate PID Controllers
    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    // Configure Drive motor
    m_drivingSpark.configure(
      Configs.MAXSwerveModule.drivingConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Configure Turn motor
    m_turningSpark.configure(
      Configs.MAXSwerveModule.turningConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // save angular offset for future use
    m_chassisAngularOffset = chassisAngularOffset;

    // set wheel angle to 0
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

    // set drive encoder position to 0
    m_drivingEncoder.setPosition(0);

  } // end  MAXSwerveModule()

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    SmartDashboard.putString("Swerve State", correctedDesiredState.toString());
    
    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  // https://github.com/FRC1257/2025-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/Module.java
  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Controllers run in "periodic" when the setpoint is not null
    // Optimize the reference state to avoid spinning further than 90 degrees.
    state.optimize(getAngle());
    
    return state;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Open loop drive control
    m_drivingSpark.setVoltage(volts);
  }

  public void runCharacterization(double driveVolts, double angleVolts) {
    // Open loop drive control
    m_drivingSpark.setVoltage(driveVolts);
    m_turningSpark.setVoltage(angleVolts);
  }

  public Rotation2d getAngle(){
    return new Rotation2d(m_turningEncoder.getPosition());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return m_drivingEncoder.getVelocity() / (ModuleConstants.kWheelDiameterMeters / 2);
  }
// ------------------------------------------------------------------------------------------------

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    // convert from meters to radians
    return m_drivingEncoder.getPosition() / ModuleConstants.kWheelRadiusMeters;
  }

}
