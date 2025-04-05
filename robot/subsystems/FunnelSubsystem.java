// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;

public class FunnelSubsystem extends SubsystemBase {
  /** Creates a new FunnelSubsystem. */
  private SparkMax entryIntakeMotor =
      new SparkMax(CoralSubsystemConstants.kEntryCoralMotorCanId, MotorType.kBrushless);
      
  public FunnelSubsystem() {
    // Configure entry motor
    entryIntakeMotor.configure(
        Configs.CoralSubsystem.entryIntakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  public void setEntryPower(double power) {
    //
    entryIntakeMotor.set(power);
  }

  public Command runEntryIntake() {
    return this.startEnd(
        () -> this.setEntryPower(-.4), () -> this.setEntryPower(0.0));
  }

  public Command runEntryOuttake() {
    return this.startEnd(
        () -> this.setEntryPower(.1), () -> this.setEntryPower(0.0));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
