// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public SparkMax armMotor =
  new SparkMax(AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
// arm's PID controller
public SparkClosedLoopController armController = armMotor.getClosedLoopController();
// arm's encoder
public RelativeEncoder armEncoder = armMotor.getEncoder();

 // Member variables for subsystem state management
 private boolean stowWhenIdle = true;
 private boolean wasReset = false;

  //Constructor
  public ArmSubsystem() {
     // configure arm motor
     armMotor.configure(
      Configs.AlgaeSubsystem.armConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

      // Zero arm encoder on initialization
    armEncoder.setPosition(0);
  }

  /** Command to force the subsystem into its "stow" state. */
  public Command stowCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = true;
        });
  }

  public void setArmPower(double power) {
    armMotor.set(power);
  }


  /** Set the arm motor position. This will use closed loop position control. */
  /*private void setIntakePosition(double position) {
    armController.setReference(position, ControlType.kPosition);
  }*/

  public void setArmToGround(){
    if(this.armEncoder.getPosition()>0){
      this.armMotor.set(0);
    }
    else {
        this.armMotor.set(0); 
    };
  }

  public void setArmLevel2(){
    if(this.armEncoder.getPosition()>0){
      this.armMotor.set(0);
    }
    else {
        this.armMotor.set(0); 
    };
  }




  public void setArmLevelMax(){


    if(this.armEncoder.getPosition()>-14.5){
      this.armMotor.set(-.3);

      this.armMotor.set(.1);
    }
    else {
        this.armMotor.set(0); 
    };
  }

  public void setArmToOuttake(){
    if(this.armEncoder.getPosition()>0){
      this.armMotor.set(0);
    }
    else {
        this.armMotor.set(0); 
    };
  }

  public double getArmPosition(){
    return armEncoder.getPosition();
  }
  public void algaeKnockHeight(){
    if(this.armEncoder.getPosition()>-12){
        this.armMotor.set(-.5);
    }
    else {
      this.armMotor.set(0);
    }
  }

   // end AlgaeSubsystem()

  

  @Override
  public void periodic() {

    // Display subsystem values
    SmartDashboard.putNumber("Algae/Arm/Position", armEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}

