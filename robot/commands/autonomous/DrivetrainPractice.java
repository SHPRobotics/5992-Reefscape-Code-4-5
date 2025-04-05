// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DrivetrainPractice extends PathPlannerAuto {
    //Constructor
    public DrivetrainPractice(RobotContainer robotContainer){
        super(Commands.run(() -> {}));

    try {
      // Load a path from a path file in storage
      PathPlannerPath TestPath = PathPlannerPath.fromPathFile("TestPath");

      // Get the ideal starting state of this path
      Pose2d startingPose =
          //new Pose2d(TestPath.getPoint(0).position, TestPath.getIdealStartingState().rotation());
            new Pose2d(0, 0, new Rotation2d(0));
            
      // Create a trigger that is high when this auto is running, and low when it is not running
      // This trigger will reset the odometry at starting pose, then follow the path
      isRunning()
          .onTrue(
              Commands.sequence(
                  AutoBuilder.resetOdom(startingPose), 
                  AutoBuilder.followPath(TestPath)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }

    }
}
