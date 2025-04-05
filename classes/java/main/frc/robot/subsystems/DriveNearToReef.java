package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

//import static org.mort11.config.constants.PIDConstants.Drivetrain.ANGLE_CONSTRAINTS;
//import static org.mort11.config.constants.PIDConstants.Drivetrain.POS_CONSTRAINTS;
//import static org.mort11.config.constants.PhysicalConstants.Drivetrain.IMU_TO_ROBOT_FRONT_ANGLE;
//import static org.mort11.config.constants.PhysicalConstants.Vision.CAMERA_RIGHT_OFFSET;
//import static org.mort11.config.constants.PhysicalConstants.Vision.CAMERA_LEFT_OFFSET;

import frc.Utility;
//import org.mort11.config.constants.PortConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain;

public class DriveNearToReef extends Command {
    private drivetrain drivetrain;
    private Vision vision;
  
    private Timer timer;
  
    private boolean isRight;
  
    public DriveNearToReef(boolean isRight) {
      // Use addRequirements() here to declare subsystem dependencies.
      drivetrain =  drivetrain.getInstance();
      vision = Vision.getInstance();
  
      timer = new Timer();
  
      this.isRight = isRight;
  
      addRequirements(drivetrain, vision);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      timer.reset();
      timer.start();
  
      drivetrain.getXController().reset(vision.getRelativeRobotPosition().getX());
        drivetrain.getYController().reset(vision.getRelativeRobotPosition().getY());
        drivetrain.getRotateController().reset(vision.getRelativeRobotPosition().getRotation().getDegrees());
      // drivetrain.getRotateController().reset(vision.getPicturePosition()[0]);
  
      drivetrain.getYController().calculate(vision.getRelativeRobotPosition().getY(), -0.3);
      drivetrain.getXController().calculate(vision.getRelativeRobotPosition().getX(), isRight ? Units.inchesToMeters(4.5) : Units.inchesToMeters(-8.5));
      drivetrain.getRotateController().calculate(vision.getPicturePosition()[0], 0);
  
      drivetrain.getXController().setConstraints(new Constraints(2,  new Constraints(1, 3).maxAcceleration));//call POS Constant
      drivetrain.getYController().setConstraints(new Constraints(2,  new Constraints(1, 3).maxAcceleration));//Thid too
      drivetrain.getRotateController().setConstraints(new Constraints(100, new Constraints(100, 180).maxAcceleration));
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      System.out.println(vision.getRelativeRobotPosition().toString());
  
      double xValue = isRight ? Units.inchesToMeters(4.5) : Units.inchesToMeters(-8.5);
  
      if(vision.hasTag()) {
  
        drivetrain.setDrive(
          new ChassisSpeeds(
            -Utility.clamp(drivetrain.getYController().calculate(vision.getRelativeRobotPosition().getY(), -0.3), 2),
            Utility.clamp(drivetrain.getXController().calculate(vision.getRelativeRobotPosition().getX(), xValue), 2),
            // -Utility.clamp(drivetrain.getRotateController().calculate(vision.getPicturePosition()[0], 0), 6)
            Utility.clamp(drivetrain.getRotateController().calculate(vision.getRelativeRobotPosition().getRotation().getDegrees(), 0), 6)
          )
        );
      }
    
      else {
        drivetrain.getXController().reset(vision.getRelativeRobotPosition().getX());
        drivetrain.getYController().reset(vision.getRelativeRobotPosition().getY());
        drivetrain.getRotateController().reset(vision.getRelativeRobotPosition().getRotation().getDegrees());
        // drivetrain.getRotateController().reset(vision.getPicturePosition()[0]);
    
        drivetrain.setDrive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, 0,
            drivetrain.getRotation2d()
          )
        );
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      drivetrain.setDrive(
        new ChassisSpeeds(0, 0, 0)
      );
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return drivetrain.getXController().atSetpoint() 
        // && drivetrain.getYController().atSetpoint() 
        // && drivetrain.getRotateController().atSetpoint();
          // return drivetrain.getXController().getPositionError() < 0.03 &&
          // drivetrain.getYController().getPositionError() < 0.03 &&
          // drivetrain.getRotateController().getPositionError() < 3;
  
          // return isAtGoal;
  
          return timer.get() > 1.3 &&
          drivetrain.getSpeed().vxMetersPerSecond < 0.01 &&
          drivetrain.getSpeed().vxMetersPerSecond < 0.01 &&
          drivetrain.getSpeed().omegaRadiansPerSecond < 0.3;
    }