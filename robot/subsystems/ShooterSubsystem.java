// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private SparkMax shooterMotor =
  new SparkMax(CoralSubsystemConstants.kShooterMotorCanId, MotorType.kBrushless);
  
  public RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  public ShooterSubsystem() {

    // Configure intake motor
    shooterMotor.configure(
        Configs.CoralSubsystem.shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

public void setShooterPower(double power) {
    //intakeLeftMotor.set(power);
    shooterMotor.set(power);
  }

  public Command shootCoral() {
    return this.startEnd(
        () -> this.setShooterPower(IntakeSetpoints.kForward), () -> this.setShooterPower(0.0));
        //() -> this.setShooterPower((elevatorEncoder.getPosition()>0 && elevatorEncoder.getPosition() < 35) ? 0.2 : 0.5 ), () -> this.setShooterPower(0.0));
  }

  public Command intakeCoral() {
    return this.startEnd(
        () -> this.setShooterPower(-.5), () -> this.setShooterPower(0.0));
  }

public Command slowIntakeCoral() {
    return this.startEnd(
        () -> this.setShooterPower(.3), () -> this.setShooterPower(0.0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
