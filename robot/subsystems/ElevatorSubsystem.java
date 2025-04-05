// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;


public class ElevatorSubsystem extends SubsystemBase {

  public boolean isElevatorGoUp = true;   // TVT - at the start because elevator at bottom, so we want to go up
  
  // Coral subsystem has 1 elevator motor and 1 shooter motor
  //
  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  // Elevator Motor
  public SparkMax elevatorMotor = new SparkMax(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  // Elevator PID Controller
  public SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  // Elevator Encoder
  public RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  // Elevator Absolute encoder
  //private AbsoluteEncoder elevatorAbsoluteEncoder = elevatorMotor.getAbsoluteEncoder(); 
  //MT 3/27 - Removed, no absolute encoder for elevator on robot
  /** Creates a new ElevatorSubsystem. */
  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1;

  public ElevatorSubsystem() {
// Configure elevator motor
elevatorMotor.configure(
  Configs.CoralSubsystem.elevatorConfig,
  ResetMode.kResetSafeParameters,
  PersistMode.kPersistParameters);

  // Zero elevator encoders on initialization
  elevatorEncoder.setPosition(0);
  }
   //MT - put this bracket here because this was an error? But the 0 seems to not be resetting now.
 
  public void setElevatorLevel1() {
    //isElevatorGoUp = true;
    if(this.elevatorEncoder.getPosition()> 2.1){     // elevator is above 0
      this.elevatorMotor.set(-.5);                 // so bring it down
    }
    else {
        this.elevatorMotor.set(0);           // elevator is reached bottom: stop motor
        isElevatorGoUp = true;                     // TVT - now we want to go up
        elevatorEncoder.setPosition(0);   // TVT - Reset Encoder position to 0 at bottom 
        
        // 3/27 MT - unable to go down because encoder value is not zero at bottom, chain is skipping so encoder value erroring
    }
  }

  public void setElevatorLevel2() {
    if (elevatorEncoder.getPosition()>21) isElevatorGoUp = false;

    System.out.println("isElevatorGoUp= "+isElevatorGoUp+", position='="+elevatorEncoder.getPosition());
      if (isElevatorGoUp){
        if(this.elevatorEncoder.getPosition()<21){    // we are going up and position < 35
          this.elevatorMotor.set(.5);           // so continue to go up
        }
        else {
              this.elevatorMotor.set(0);        // we are going up and pass position 35, so stop the motor
        };
      }
      else {
        if(this.elevatorEncoder.getPosition()>21){    // we are going down and position is above 35 
          this.elevatorMotor.set(-.5);                //   so continue to go down to reach 35
        }
        else {
              this.elevatorMotor.set(0);        // we are going down and position is < 35, stop the motor
              isElevatorGoUp = true;                  // now we want it to go up to reach 35
        };
  
      }
  
    }


    public void setElevatorLevel3() {
      if (elevatorEncoder.getPosition()>21) isElevatorGoUp = false;

      System.out.println("isElevatorGoUp= "+isElevatorGoUp+", position='="+elevatorEncoder.getPosition());
        if (isElevatorGoUp){
          if(this.elevatorEncoder.getPosition()<50){    // we are going up and position < 49
            this.elevatorMotor.set(.5);           // so continue to go up
          }
          else {
                this.elevatorMotor.set(0);        // we are going up and pass position 49, so stop the motor
          };
        }
        else {
          if(this.elevatorEncoder.getPosition()>50){    // we are going down and position is above 49 
            this.elevatorMotor.set(-.5);                //   so continue to go down to reach 49
          }
          else {
                this.elevatorMotor.set(0);        // we are going down and position is < 49, stop the motor
                isElevatorGoUp = true;                  // now we want it to go up to reach 49
          };
        }
       }
    // 3/27 MT - still not working in teleop. Does this code affect setreference in RobotContainer? It's going too far for L3
    
public void setElevatorLevel4() {
  //isElevatorGoUp = false;

  if(this.elevatorEncoder.getPosition()<95){    // we are below 92
    this.elevatorMotor.set(0.5);           // so continue to go up
  }
  
  else {
      this.elevatorMotor.set(0);          // we are above 92: stop motor
      isElevatorGoUp = false;                   // TVT - now we want to go down
    };
}

public void setElevatorSpeed(double speed) {
  elevatorMotor.set(speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator RelEncoder Position", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator go up?", isElevatorGoUp);
    //SmartDashboard.putData((Sendable) elevatorClosedLoopController);
  }
}
