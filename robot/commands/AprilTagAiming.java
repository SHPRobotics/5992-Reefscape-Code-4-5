// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AprilTagAiming extends Command {

  DriveSubsystem swerveDrive;
  LimelightSubsystem limelight;

  PIDController pidController;

  int blueTag;
  int redTag;

  // This value will change depending on the weight of the robot.
  double Kp = .002185; //-0.0076;

  /** Creates a new AprilTagAiming. */
  public AprilTagAiming(DriveSubsystem swerveDrive, LimelightSubsystem limelight, int blueTag, int redTag) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
  
    // pidController = new PIDController(0.001, .09, 0);

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

    //Gets Horizontal Offset From Crosshair To Target
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
System.out.print("tx = " + tx);
System.out.print(", tid = " + tid);

    // If robot starts at middle position, aim tag #21 for Blue Alliance or tag #10 for Red Alliance
    if (tid == blueTag || tid == redTag) {

      /* rot: the rotation/angular speed of the robot. As the tx value gets smaller (meaning the horizontal
      offset from the crosshair to the target gets closer to the center of the camera), the angular speed
      decreases.
      */
      double rot = Kp * tx * 2;

      /* If the limelight overshoots the target to a point it cannot read it, it kept on spinning.
      So, this conditional statement will make it stop spinning if it overshoots it and can't detect
      the April Tag anymore.
      */ 
      if (limelight.getIsDetecting() == false) {
        rot = 0;
      }
System.out.println(", rot = " + rot);

      // double rotSpeed = pidController.calculate(headingError, 0);
      swerveDrive.drive(0, 0, -rot, false);
    }
    else {
      System.out.println("");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.turnOffLED();
    swerveDrive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
