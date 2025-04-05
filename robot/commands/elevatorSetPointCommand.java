// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorSetPointCommand extends Command {
  private final ElevatorSubsystem m_elvatorSubsystem;
  private double m_setpoint;

  private final double kP = 0.5;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double iLimit = 1.0;

  private double error = 0.0;
  private double errorSum = 0.0;
  private double lastTimestamp = 0.0;
  private double lastError = 0.0;
  private double tolerance = 0.1;

  /** Creates a new setSetPointCommand. */
  public elevatorSetPointCommand(ElevatorSubsystem subsystem, double setPoint) {
    m_elvatorSubsystem = subsystem;
    m_setpoint = setPoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elvatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elvatorSubsystem.elevatorEncoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get sensor position
    double sensorPosition = m_elvatorSubsystem.elevatorEncoder.getPosition();

    // calculations
    error = m_setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    double output = kP * error + kI * errorSum + kD * errorRate;

    // output to motor
    m_elvatorSubsystem.elevatorMotor.set(output * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elvatorSubsystem.elevatorMotor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= tolerance;
  }
}
