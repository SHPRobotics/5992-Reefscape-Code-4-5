// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class GettingInRangeAT extends Command {
  double Kp = .045; //0.5000;
  // double Kp_rot = -.0076;
  double maxSpeed = .4;

  double currentDistanceZ;
  double desiredDistance;
  int blueTag;
  int redTag;

  DriveSubsystem swerveDrive;
  LimelightSubsystem limelight;

  /** Creates a new GettingInRangeAT. */
  public GettingInRangeAT(DriveSubsystem swerveDrive, LimelightSubsystem limelight, double desiredDistance, int blueTag, int redTag) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
    this.desiredDistance = desiredDistance;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.turnOnLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

    double[] botpose_targetspace = limelight.getBotPositionToTargetSpace();
    double currentDistanceZ = -botpose_targetspace[2];
    double currentDistanceX = -botpose_targetspace[0];
    double distance = Math.hypot(currentDistanceX, currentDistanceZ);
System.out.print("tv = " + tv);
System.out.print(", tid = " + tid);

    double distanceError = desiredDistance - distance;
    if (tid == blueTag || tid== redTag) {

      double speedX = distanceError * Kp;
      // double speedY = distanceErrorX * Kp;
      // double rot = tx * Kp_rot;

      if (tv == 0) {
        speedX = 0;
        // speedY = 0;
        // rot = 0;
      }
      
      // I don't want the robot to go a very fast speedY/X if it is very far away from robot
      if (Math.abs(speedX) > maxSpeed) {
        speedX = maxSpeed;
      }

      // if (Math.abs(speedY) > maxSpeed) {
      //   speedY = maxSpeed;
      // }

      // if (Math.abs(distanceErrorX) < 0.1) {
      //   speedY = 0;
      // }

      if (Math.abs(distanceError) < 0.1) {
        speedX = 0;
      }
System.out.println(", speedX = " + speedX);    
      swerveDrive.drive(speedX, 0, 0, false);
    }
    else {
      System.out.println("");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.turnOnLED();
    swerveDrive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
