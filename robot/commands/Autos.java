// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  // Tony's Button(s)
public static Command LL_Left_Align(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem){
  return Commands.sequence(
    // limelight().withTimeout(1.5)
    new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                      0,
                                              driveSubsystem.limelight_aim_and_range()[0],
                                                                                  false),
                                                                          driveSubsystem)
                                                                        .withTimeout(1.5),
    // wait
    new WaitCommand(.5),
//leftStrafeAuto()
    Autos.StrafeDistanceAuto(driveSubsystem, false, .18),//.2159
    
//wait
    new WaitCommand(.25),

// setElevatorSpeed(1)
    new RunCommand(()->elevatorSubsystem.setElevatorSpeed(1.0))
    .until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87));
}

public static Command LL_Right_Align(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem){
  return Commands.sequence(
    // limelight().withTimeout(1.5)
    new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                      0,
                                              driveSubsystem.limelight_aim_and_range()[0],
                                                                                  false),
                                                                          driveSubsystem)
                                                                        .withTimeout(1),
    // wait
    new WaitCommand(.5),
//rightStrafeAuto()
    Autos.StrafeDistanceAuto(driveSubsystem, true, .13),
    
//wait
    new WaitCommand(.25),

// setElevatorSpeed(1)
    new RunCommand(()->elevatorSubsystem.setElevatorSpeed(1.0))
    .until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87));
}
//

//Auton Commands
public static Command LL_Mid_OneCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem){
  return Commands.sequence(
    // limelight().withTimeout(1.5)
    new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                      0,
                                              driveSubsystem.limelight_aim_and_range()[0],
                                                                                  false),
                                                                          driveSubsystem)
                                                                        .withTimeout(1.5),
    // wait
    new WaitCommand(.5),

    //leftStrafeAuto()
    Autos.StrafeDistanceAuto(driveSubsystem, false, .18),
    
//wait
    new WaitCommand(.25),


// setElevatorSpeed(1)
new RunCommand(()->elevatorSubsystem.setElevatorSpeed(.50))
.until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87)
.andThen(()->{shooterSubsystem.setShooterPower(.4);})
    );
}

public static Command LL_Side_TwoCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem, FunnelSubsystem funnelSubsystem){
  return Commands.sequence(
    // deploy limelight to approach reef
    new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                      0,
                                              driveSubsystem.limelight_aim_and_range()[0],
                                                                                  false),
                                                                          driveSubsystem)
                                                                        .withTimeout(1.5),
// wait
    new WaitCommand(.5),

//strafe left at the reef
new RunCommand(()->Autos.StrafeDistanceAuto(driveSubsystem, false, .18)),
    
//wait
    new WaitCommand(.5),

// deploy elevator and shooter
    new RunCommand(()->elevatorSubsystem.setElevatorSpeed(.850))
    .until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87)
    .andThen(()->{shooterSubsystem.setShooterPower(.4);}),

// wait

    new WaitCommand(1),

// holster elevator and stop shooter
    new RunCommand(()->shooterSubsystem.setShooterPower(0)),

    new RunCommand(()->elevatorSubsystem.setElevatorLevel1()).withTimeout(2),

// back up from reef 
    new RunCommand(()-> Autos.DriveAngleDistanceAuto(driveSubsystem, -1.0, driveSubsystem.getHeading())),

// wait
    new WaitCommand(.25),

// perform 180 deg turn

    new RunCommand(()-> driveSubsystem.drive(0, 0, .3, false))
        .until(()->driveSubsystem.getHeading()<55), //experimental value required

// wait
    new WaitCommand(.5),

// track back to feeder station
  new RunCommand(()-> Autos.DriveAngleDistanceAuto(driveSubsystem, -4.44, driveSubsystem.getHeading())),

// run funnel
  new RunCommand(()-> funnelSubsystem.setEntryPower(-.4)).withTimeout(2),

// deploy limelight to approach reef
  new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                     0,
                                           driveSubsystem.limelight_aim_and_range()[0],
                                                                                false),
                                                                        driveSubsystem)
                                                                      .withTimeout(1.5),
// wait
new WaitCommand(.5),

//strafe left at the reef
new RunCommand(()->Autos.StrafeDistanceAuto(driveSubsystem, false, .18)),
    
//wait
    new WaitCommand(.25),

// deploy elevator and shooter
    new RunCommand(()->elevatorSubsystem.setElevatorSpeed(1.0))
    .until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87)
    .andThen(()->{shooterSubsystem.setShooterPower(.4);})
    .withTimeout(1)
  );
}

public static Command LL_Side_ThreeCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem, FunnelSubsystem funnelSubsystem){
  return Commands.sequence(
    // deploy limelight to approach reef
    new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                      0,
                                              driveSubsystem.limelight_aim_and_range()[0],
                                                                                  false),
                                                                          driveSubsystem)
                                                                        .withTimeout(1.5),
// wait
    new WaitCommand(.5),

//strafe left at the reef
new RunCommand(()->Autos.StrafeDistanceAuto(driveSubsystem, false, .18)),
    
//wait
    new WaitCommand(.25),

// deploy elevator and shooter
    new RunCommand(()->elevatorSubsystem.setElevatorSpeed(1.0))
    .until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87)
    .andThen(()->{shooterSubsystem.setShooterPower(.4);}),

// wait

    new WaitCommand(1),

// holster elevator and stop shooter
    new RunCommand(()->shooterSubsystem.setShooterPower(0)),

    new RunCommand(()->elevatorSubsystem.setElevatorLevel1()).withTimeout(2),

// back up from reef 
    new RunCommand(()-> Autos.DriveAngleDistanceAuto(driveSubsystem, -1.0, driveSubsystem.getHeading())),

// wait
    new WaitCommand(.25),

// perform 180 deg turn

    new RunCommand(()-> driveSubsystem.drive(0, 0, .3, false))
        .until(()->driveSubsystem.getHeading()<55), //experimental value required

// wait
    new WaitCommand(.5),

// track back to feeder station
  new RunCommand(()-> Autos.DriveAngleDistanceAuto(driveSubsystem, -4.44, driveSubsystem.getHeading())),

// run funnel
  new RunCommand(()-> funnelSubsystem.setEntryPower(-.4)).withTimeout(2),

// deploy limelight to approach reef
  new RunCommand(()-> driveSubsystem.drive(driveSubsystem.limelight_aim_and_range()[1],
                                                                                     0,
                                           driveSubsystem.limelight_aim_and_range()[0],
                                                                                false),
                                                                        driveSubsystem)
                                                                      .withTimeout(1.5),
// wait
new WaitCommand(.5),

//strafe left at the reef
new RunCommand(()->Autos.StrafeDistanceAuto(driveSubsystem, false, .18)),
    
//wait
    new WaitCommand(.25),

// deploy elevator and shooter
    new RunCommand(()->elevatorSubsystem.setElevatorSpeed(1.0))
    .until(()-> elevatorSubsystem.elevatorEncoder.getPosition()>87)
    .andThen(()->{shooterSubsystem.setShooterPower(.4);})
    .withTimeout(1)
  );
}


// Auton commands --------------------------------------------------------------------------------------------------
  // DriveDistanceAuto(): drive forward (driveReversed=false) / backward (driveReversed=true) a specified distance then stop
  public static Command DriveDistanceAuto(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
        driveSubsystem::resetEncoders,

      // onExecute: drive forward (if driveReversed = false) or reverse (if driveReversed = true) at an angle while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
      () -> driveSubsystem.drive(-AutoConstants.kAutoDriveSpeed * (driveReversed ? -1 : 1) , // Forward
                                 0,                                                  // Strafe
                                 0,                                                     // Rotation
                                 false),

      // onEnd: stop driving at the end of command
      interrupt -> driveSubsystem.setX(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      () -> Math.abs(driveSubsystem.getAverageEncoderDistance()) >= Math.abs((distanceMeters)),
      
      // require the drive subsystem
      driveSubsystem);
  }

  public static Command StrafeDistanceAuto(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
        driveSubsystem::resetEncoders,

      // onExecute: drive forward (if driveReversed = false) or reverse (if driveReversed = true) at an angle while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
      () -> driveSubsystem.drive(0, // Forward
      -AutoConstants.kAutoDriveSpeed * (driveReversed ? -1 : 1),                                                  // Strafe
                                 0,                                                     // Rotation
                                 false),

      // onEnd: stop driving at the end of command
      interrupt -> driveSubsystem.setX(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      () -> Math.abs(driveSubsystem.getAverageEncoderDistance()) >= Math.abs((distanceMeters)),
      
      // require the drive subsystem
      driveSubsystem);
  }

  // DriveAngleDistanceAuto(): strafe forward (distanceMeters>0) / backward (distanceMeters<0) at a specified angle and distance then stop
  public static Command DriveAngleDistanceAuto(DriveSubsystem driveSubsystem, double distanceMeters, double driveAngleDeg){
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

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
