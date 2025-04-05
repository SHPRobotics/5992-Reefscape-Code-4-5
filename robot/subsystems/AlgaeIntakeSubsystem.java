// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor =
  new SparkMax(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  private boolean stowWhenIdle = true;
  private boolean wasReset = false;
  /** Creates a new AlgaeIntakeSubsystem. */

  //Constructor
  public AlgaeIntakeSubsystem() {
  // Algae subststem has 1 arn motor and 1 intake notor
    // Configure intake motor
    intakeMotor.configure(
        Configs.AlgaeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setArmWheelPower(double power) {
    intakeMotor.set(power);
  }

  public Command intakeAlgae() {
    return this.startEnd(
         () -> this.setArmWheelPower(-.7), () -> this.setArmWheelPower(0.0));  // TVT
  /*
        // TVT
        () -> {
                if (armEncoder.getPosition() > -41)    // TO DO: Test this value
                  this.setArmPower(AlgaeSubsystemConstants.IntakeSetpoints.kForward);
                else 
                  setArmPower(0);
                
              }, () -> this.setArmPower(0.0));
        // TVT
  */
  }
/*
  public Command stopIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = false;                                               // currently not holding algae
          setArmWheelPower(AlgaeSubsystemConstants.IntakeSetpoints.kStop);   // take algae in at half speed
          //setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kDown);      // set arm position down to 0 position
        });
  }
*/

  public Command shootAlgae() {
  return this.startEnd(
    () -> this.setArmWheelPower(.7), () -> this.setArmWheelPower(0.0));
  }

  public Command stowCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = true;
        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae/Intake/Applied Output", intakeMotor.getAppliedOutput());
  }
}


