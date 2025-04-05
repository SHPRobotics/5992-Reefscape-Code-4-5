// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagAiming;
import frc.robot.commands.Autos;
import frc.robot.commands.GettingInRangeAT;
import frc.robot.commands.elevatorSetPointCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkBase.ControlType;

/*
 * ================================================================================================
 * BUTTONS BINDING
 * ================================================================================================
 * DRIVER (JOYSTICK 0)
 * ------------------------------------------------------------------------------------------------
 * Press Y button       -> Toggle slowMode (e.g.: drive full speed to half speed and vice versa)
 * Press Start Button   -> Zero swerve heading
 * Hold X button        -> stop drivetrain and lock wheels in X positions
 * Hold left bumper     -> strafe true left
 * Hold right bumper    -> strafe true right
 * Hold A button        -> aim and range target using limelight
 * Hold B button        -> drive the robot a specified distance in front of target
 * Hold Left Trigger    -> Aim at April Tag #21 or #10 and move to position 1.25 m away from April Tag
 * Hold Right Trigger   -> Aim at April Tag #21 or #10
 * Hold Back button     -> Blinking LED
 * ------------------------------------------------------------------------------------------------
 * OPERATOR (JOYSTICK 1) 
 * ------------------------------------------------------------------------------------------------
 * CORAL SUBSYSTEM
 * ------------------------------------------------------------------------------------------------
 * Left Trigger         -> Run tube intake
 * Right Trigger        -> Run tube intake in reverse
 * A Button             -> Elevator/Arm to level 1 position
 * X Button             -> Elevator/Arm to level 2 position
 * B Button             -> Elevator/Arm to level 3 position
 * Y Button             -> Run entry intake
 * ------------------------------------------------------------------------------------------------
 * ALGAE SUBSYSTEM
 * ------------------------------------------------------------------------------------------------
 * Right Bumper          -> Run ball intake, set to leave out when idle
 * Left Bumper         -> Run ball intake in reverse, set to stow when idle
 * 
*/

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's Drivesubsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    // Coral subsystem
    //private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

    private final FunnelSubsystem m_funnelSubsystem = new FunnelSubsystem();

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    //private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    // Algae subsystem
    //private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

    private final AlgaeIntakeSubsystem m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    // Limelight subsystem
    private final LimelightSubsystem m_Limelight = new LimelightSubsystem();
    // LED subsystem
    private final LED m_led = new LED();
    //Auto Subsystem OC
   // private final Autos m_Autos = new Autos();
    // Color Sensor subsystem
    //private final ColorSensorSubsystem m_ColorSensor = new ColorSensorSubsystem();

    // The robot's commands
    // Objects for the aprilTagAiming command for alll possible positions in the reef
    private final AprilTagAiming aprilTagAiming20and11 = new AprilTagAiming(m_robotDrive, m_Limelight, 20, 11);
    private final AprilTagAiming aprilTagAiming21and10 = new AprilTagAiming(m_robotDrive, m_Limelight, 21, 10);
    private final AprilTagAiming aprilTagAiming22and9 = new AprilTagAiming(m_robotDrive, m_Limelight, 22, 9);
    private final AprilTagAiming aprilTagAiming17and8 = new AprilTagAiming(m_robotDrive, m_Limelight, 17, 8);
    private final AprilTagAiming aprilTagAiming18and7 = new AprilTagAiming(m_robotDrive, m_Limelight, 18, 7);
    private final AprilTagAiming aprilTagAiming19and6 = new AprilTagAiming(m_robotDrive, m_Limelight, 19, 6);

    // Objects for the gettingInRange command for alll possible positions in the reef
    private final GettingInRangeAT gettingInRangeAT20and11 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.25, 20, 11);
    private final GettingInRangeAT gettingInRangeAT21and10 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.25, 21, 10);
    private final GettingInRangeAT gettingInRangeAT22and9 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.25, 22, 9);
    private final GettingInRangeAT gettingInRangeAT17and8 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.25, 17, 8);
    private final GettingInRangeAT gettingInRangeAT18and7 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.25, 18, 7);
    private final GettingInRangeAT gettingInRangeAT19and6 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.25, 19, 6);
    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // The operator's controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    // Declare AUTO Commands ---------------------------------------------------------------------------------------------------------
    // auto command to drive the robot forward for 2 meters
    private final Command m_DriveDistanceAuto = Autos.DriveDistanceAuto(m_robotDrive, false, 2.0);
    // auto Command to strafe forward 1 meters at a 5 deg angle
     private final Command m_StrafeFwd1MetersAt5Deg = Autos.DriveAngleDistanceAuto(m_robotDrive, 1.0, 5.0);
    // auto Command to strafe reverse 3 meters at angle 45 deg.
    private final Command m_StrafeRev3MetersAt45Deg = Autos.DriveAngleDistanceAuto(m_robotDrive, -3.0, 45.0);
    // Command to strafe specific distance for reef alignment
    
    private final Command m_RightReefAlign = DriveSubsystem.DriveAngleDistance(m_robotDrive, .20, 90);
    private final Command m_LeftReefAlign = DriveSubsystem.DriveAngleDistance(m_robotDrive, -.20, 90);
    // Declare a sendable chooser (similar to a dropdown list) for user to select which autonomous command to run
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Register named commands. NOTE: we must register named commands and event triggers before build the autochooser
        // https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/RobotContainer.java
        // NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        // NamedCommands.registerCommand("print hello", Commands.print("hello"));
        NamedCommands.registerCommand("Limelight", new RunCommand(()-> m_robotDrive.drive(
            m_robotDrive.limelight_aim_and_range()[1], 
            0.0, 
            m_robotDrive.limelight_aim_and_range()[0], 
            false), 
            m_robotDrive).withTimeout(1.5));

        //NamedCommands.registerCommand("shootCoral", new RunCommand(()-> {m_shooterSubsystem.shootCoral();}));
        NamedCommands.registerCommand("Shoot Coral", new RunCommand(()->m_shooterSubsystem.setShooterPower(.4)).withTimeout(.5));
        //NamedCommands.registerCommand("Practice Shot", new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(.5)).until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>35).andThen(()->m_shooterSubsystem.setShooterPower(.5)));
        
        NamedCommands.registerCommand("Practice Shot L2", new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(1))
                                                                .until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>18)
                                                                .andThen(()->{m_shooterSubsystem.setShooterPower(.7);
                                                                            }));
        NamedCommands.registerCommand("Practice Shot L4", new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(1))
                                                                            .until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>87)
                                                                            .andThen(()->{m_shooterSubsystem.setShooterPower(.4);
                                                                                        }));

        NamedCommands.registerCommand("Practice Shot L3", new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(1))
                                                                .until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>50)
                                                                .andThen(()->{m_shooterSubsystem.setShooterPower(.7);
                                                                            }));


        NamedCommands.registerCommand("PID Practice Shot", new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(90.50,ControlType.kPosition))
                                                                .andThen(new WaitCommand(1.5)));
                                                                //.andThen(()->{m_shooterSubsystem.setShooterPower(.7);}));
                                                                            
        //NamedCommands.registerCommand("Practice Shot", new RunCommand(()->m_shooterSubsystem.setShooterPower(.7)));
        
        NamedCommands.registerCommand("Stop Shooter", new RunCommand(()->m_shooterSubsystem.setShooterPower(0)));                                                                    

        NamedCommands.registerCommand("intakeCoral", Commands.runOnce(()-> {m_shooterSubsystem.intakeCoral();}));
        //NamedCommands.registerCommand("entry Coral", Commands.runOnce(()-> {m_funnelSubsystem.runEntryIntake();}));
        NamedCommands.registerCommand("Entry Coral", new RunCommand(()-> {m_funnelSubsystem.setEntryPower(-.4);}).withTimeout(1));
        NamedCommands.registerCommand("Stop Entry", new RunCommand(()->m_funnelSubsystem.setEntryPower(0)));
        NamedCommands.registerCommand("setElevatorL1", new RunCommand(()->{m_elevatorSubsystem.elevatorClosedLoopController.setReference(0.0,ControlType.kPosition);}));
        //NamedCommands.registerCommand("setElevatorL2", new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel2();}));
        NamedCommands.registerCommand("setElevatorL3", new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel3();}));
        NamedCommands.registerCommand("setElevatorL4", new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel4();}));

        //NamedCommands.registerCommand("raiseArm", new RunCommand(()-> {m_armSubsystem.setArmLevelMax();}));
        NamedCommands.registerCommand("raiseArm", new RunCommand(()->m_armSubsystem.setArmPower(.5))
                                                            .until(()->m_armSubsystem.armEncoder.getPosition()<-14.5));

        NamedCommands.registerCommand("PID raiseArm",new RunCommand(()->{m_armSubsystem.armController.setReference(-13,ControlType.kPosition);}));
        NamedCommands.registerCommand("PID lowArm",new RunCommand(()->{m_armSubsystem.armController.setReference(0,ControlType.kPosition);}));
        NamedCommands.registerCommand("algaeOut", new RunCommand(()-> {m_algaeIntakeSubsystem.setArmWheelPower(-.5);}));
        NamedCommands.registerCommand("algaeIn", new RunCommand(()-> {m_algaeIntakeSubsystem.setArmWheelPower(.5);}));
        NamedCommands.registerCommand("algaeStop", new RunCommand(()->{m_algaeIntakeSubsystem.setArmWheelPower(0);}));
        /*NamedCommands.registerCommand("setElevatorL1", m_coralSubsystem.setElevatorSetpoint("kLevel1"));
        NamedCommands.registerCommand("setElevatorL2", m_coralSubsystem.setElevatorSetpoint("kLevel2"));
        NamedCommands.registerCommand("setElevatorL3", m_coralSubsystem.setElevatorSetpoint("kLevel3"));*/
        /*NamedCommands.registerCommand("setArmToGround", new RunCommand(()->{m_algaeSubsystem.setArmToGround();}));*
        NamedCommands.registerCommand("setArmL2", new RunCommand(()->{m_algaeSubsystem.setArmLevel2();}));
        NamedCommands.registerCommand("setArmL3", new RunCommand(()->{m_algaeSubsystem.setArmLevel3();}));
        NamedCommands.registerCommand("setArmOuttake", new RunCommand(()->{m_algaeSubsystem.setArmToOuttake();}));*/
        /*NamedCommands.registerCommand("setArmPickupOnCoral", m_algaeSubsystem.setArmSetpoint(Setpoint.kLevelGround));
        NamedCommands.registerCommand("setArmGroundPickup", m_algaeSubsystem.setArmSetpoint(Setpoint.kLevelOnCoral));
        NamedCommands.registerCommand("setArmL2", m_algaeSubsystem.setArmSetpoint(Setpoint.kLevel2));
        NamedCommands.registerCommand("setArmL3", m_algaeSubsystem.setArmSetpoint(Setpoint.kLevel3));
        NamedCommands.registerCommand("setArmOuttake", m_algaeSubsystem.setArmSetpoint(Setpoint.kOuttake)); Owen's wait: below*/
        NamedCommands.registerCommand("wait", new RunCommand(()->{m_robotDrive.setX();}));

        // Use event markers as triggers
        // https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/RobotContainer.java
        // new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

        //Note for MJ -before thursday comp- (make a command and event trigger for limelight to reef align)

        new EventTrigger("Limelight").onTrue(new RunCommand(()-> m_robotDrive.drive(
            m_robotDrive.limelight_aim_and_range()[1], 
            0.0, 
            m_robotDrive.limelight_aim_and_range()[0], 
            false), 
            m_robotDrive).withTimeout(1.5));


        new EventTrigger("PID Practice Shot").onTrue(new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(90.50,ControlType.kPosition))
                                                           .andThen(new WaitCommand(1.5)));
                                                           //.andThen(()->{m_shooterSubsystem.setShooterPower(.7);}));

        new EventTrigger("Practice Shot L4").whileTrue(new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(1))
        .until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>87)
        .andThen(()->{m_shooterSubsystem.setShooterPower(.4);
                    }));

                    new EventTrigger("Practice Shot L2").whileTrue(new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(1))
                    .until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>18)
                    .andThen(()->{m_shooterSubsystem.setShooterPower(.4);
                                }));

                    new EventTrigger("Practice Shot L3").onTrue(new RunCommand(()->m_elevatorSubsystem.setElevatorSpeed(1))
                    .until(()->m_elevatorSubsystem.elevatorEncoder.getPosition()>50)
                    .andThen(()->{m_shooterSubsystem.setShooterPower(.7);
                                }));

        new EventTrigger("Stop Shooter").onTrue(new RunCommand(()->m_shooterSubsystem.setShooterPower(0)));
        new EventTrigger("setElevatorL1").onTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel1();}));
        new EventTrigger("setElevatorL2").onTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel2();}));
        new EventTrigger("PID setElevatorL2").onTrue(new RunCommand(()->{m_elevatorSubsystem.elevatorClosedLoopController.setReference(35.0,ControlType.kPosition);}));
        new EventTrigger("setElevatorL3").onTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel3();}));
        new EventTrigger("SetElevatorL4").onTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel4();}));
        new EventTrigger("Shoot Coral").whileTrue(new RunCommand(()-> m_shooterSubsystem.setShooterPower(0.4)).withTimeout(.5));
        new EventTrigger("Entry Coral").onTrue(new RunCommand(()->m_funnelSubsystem.setEntryPower(-.4)).withTimeout(1));
        new EventTrigger("intakeCoral").onTrue(Commands.runOnce(()-> {m_shooterSubsystem.intakeCoral();}));
        new EventTrigger("Stop Entry").onTrue(new RunCommand(()->m_funnelSubsystem.setEntryPower(0)));
      //  new EventTrigger("LimeLightAim").onTrue(new RunCommand(()->m_robotDrive.limelight_aim_and_range()));

    
        new EventTrigger("raiseArm").onTrue(new RunCommand(()->{m_armSubsystem.setArmLevelMax();}));
        new EventTrigger("PID raiseArm").onTrue(new RunCommand(()->{m_armSubsystem.armController.setReference(-13,ControlType.kPosition);}));
        new EventTrigger("PID lowArm").onTrue(new RunCommand(()->{m_armSubsystem.armController.setReference(0,ControlType.kPosition);}));
        new EventTrigger("algaeOut").onTrue(new RunCommand(()->{m_algaeIntakeSubsystem.setArmWheelPower(-.5);}));
        new EventTrigger("algaeIn").onTrue(new RunCommand(()->{m_algaeIntakeSubsystem.setArmWheelPower(.5);}));
        new EventTrigger("algaeStop").onTrue(new RunCommand(()->{m_algaeIntakeSubsystem.setArmWheelPower(0);}));

        //adding new limelight commands
        //new EventTrigger("LimelightAim").onTrue(new RunCommand(()->{m_robotDrive.limelight_aim_proportional();}));
       // new EventTrigger("LimelightAim").onTrue(new RunCommand(()->{m_robotDrive.limelight_aim_proportional();}));  
        
        //new EventTrigger("entryCoral").onTrue(Commands.runOnce(()-> {m_funnelSubsystem.runEntryIntake();}));
        /*new EventTrigger("setArmToGround").onTrue(new RunCommand(()->{m_algaeSubsystem.setArmToGround();}));
        new EventTrigger("setArmL2").onTrue(new RunCommand(()->{m_algaeSubsystem.setArmLevel2();}));
        new EventTrigger("setArmL3").onTrue(new RunCommand(()->{m_algaeSubsystem.setArmLevel3();}));
        new EventTrigger("setArmOuttake").onTrue(new RunCommand(()->{m_algaeSubsystem.setArmToOuttake();}));*/


        // Create and populate a sendable chooser with all PathPlannerAutos in the project. The default option will be Command "BlueLeaveSL_ScoreCoralL1"
        autoChooser = AutoBuilder.buildAutoChooser("LeaveSL_ScoreCoralL1");

        // Configure the button bindings
        configureButtonBindings();

        // Configure autonomous commands
        configureAutonomous();


        // Create Tab 'Autonomous' and put the autoChooser in
        Shuffleboard.getTab("Autonomous")
            .add("auto choices", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0,12)
            .withSize(2, 1);

        // Configure the dashboard
        //configureDashboard();

        // Configure default command for Drive subsystem
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
                m_robotDrive));

           
        // Configure default command for ColorSensor subsystem
        //m_ColorSensor.setDefaultCommand(ReadingRainbow());

        // Set the ball intake to in/out when not running based on internal state
        //m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

        // Default command for Algae subsystem. Move right stick up to raise the arm, down to lower the arm
        m_armSubsystem.setDefaultCommand(new RunCommand( 
            () -> {m_armSubsystem.setArmPower(MathUtil.applyDeadband(m_operatorController.getLeftY()*.7, OIConstants.kDriveDeadband));}, 
            m_armSubsystem));

        // Default command for Coral subsystem. Move left stick up to raise the elevator, down to lower the elevator
        m_elevatorSubsystem.setDefaultCommand( new RunCommand( 
            () -> {m_elevatorSubsystem.setElevatorSpeed(-MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kDriveDeadband));}, 
            m_elevatorSubsystem));

    }   // end RobotContainer()

// TVT
    public void stopAllMotors(){
        m_algaeIntakeSubsystem.setArmWheelPower(0);
        m_armSubsystem.setArmPower(0);

        m_shooterSubsystem.setShooterPower(0);
        m_funnelSubsystem.setEntryPower(0);
        m_elevatorSubsystem.setElevatorSpeed(0);
    }

// SB 3/27
    public Command resetElevator(){
        m_elevatorSubsystem.elevatorEncoder.setPosition(0);
        return Commands.none();
    }
// TVT

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
/* TVT - THESE 4 PATHPLANNER COMMANDS ARE FOR TESTING ONLY
        // ---------------------- PATHPLANER WITH BUTTON 'Run' --------------------------------------------------
        // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
        SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

        // Add a button to run pathfinding commands to SmartDashboard
        //A command to pathfind to a given pose
        SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
            new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
            new PathConstraints(
                4.0, 4.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
            ), 
            0
        ));

        SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
            new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
            new PathConstraints(
                4.0, 4.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
            ), 
            0
        ));

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m in the +X field direction
        SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
            Pose2d currentPose = m_robotDrive.getPose();
            
            // The rotation component in these poses represents the direction of travel
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                new PathConstraints(
                    4.0, 4.0, 
                    Units.degreesToRadians(360), Units.degreesToRadians(540)
                ),
                null, // Ideal starting state can be null for on-the-fly paths
                new GoalEndState(0.0, currentPose.getRotation())
            );

            // Prevent this path from being flipped on the red alliance, since the given positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        })); // end SmartDashboard.putData
TVT */
/*        
        // -------------------------- COLOR SENSOR ---------------------------------------------
        // In order to run setX or setH command, we have to schedule it first
        // To schedule a command, we have to creaste a Trigger
        // Schedule 'setX Command` when isColorRed() changes to `true`
        // Schedule 'setH Command` when isColorRed() changes to `false`
         new Trigger(m_ColorSensor::isColorRed)
           .onTrue(new RunCommand(
            .onTrue(new InstantCommand(
        () -> m_robotDrive.setH(),
                m_robotDrive))
            .onFalse(new RunCommand(
            .onFalse(new InstantCommand(
                    () -> m_robotDrive.setX(),
                m_robotDrive));

        new Trigger(m_ColorSensor::isColorRed)
                //        .onTrue(new RunCommand(
                            .onTrue(m_robotDrive.setX())
                //            .onFalse(new RunCommand(
                            .onFalse(m_robotDrive.setH());

        m_driverController.y()
            .whileTrue(new RunCommand( () -> ReadingRainbow()));

            m_driverController.y()
            .whileTrue(ReadingRainbow());
*/
        // -------------------------------- CHASSIS --------------------------------------------
        // Press right trigger -> toggle drive speed from fast to slow and vice versa
        /*m_driverController.y()
            .onTrue(new InstantCommand(DriveSubsystem::toggleSlowMode));*/

        // Start Button -> Zero swerve heading
        /*m_driverController.start()
            .onTrue(new RunCommand( 
                () -> {m_robotDrive.zeroHeading();} , 
                m_robotDrive));*/

        // Hold X button to stop drivetrain and lock wheels in X positions
        // Hold left bumper for strafe true left (Make this speed faster or slower?)
        m_driverController.leftBumper()
            .whileTrue(new RunCommand(
                ()-> m_robotDrive.drive(0, -0.15, 0, false), 
                m_robotDrive));

        // Hold right bumper for strafe true right
        m_driverController.rightBumper()
            .whileTrue(new RunCommand(
                ()-> m_robotDrive.drive(0, 0.15, 0, false), 
                m_robotDrive));
        

        // ----------------------------- CHASSIS AND LED -------------------------------------------------------
        // back button for blinking LED ?
        m_driverController.back()
        .whileTrue(new InstantCommand(
            () -> m_led.setLEDVoltage(), m_led));

        // -------------------------------- CHASSIS AND LIMELIGHT -----------------------------------------------

        // Hold left trigger to aim at April Tag #21 or #10 then move to position 1.25 m away from AprilTag
        /*m_driverController.leftTrigger()
            .whileTrue(gettingInRangeAT19and6);*/

        // Hold right trigger to aim at AprilTag #21 or #10
        /*m_driverController.rightTrigger()
            .whileTrue(aprilTagAiming19and6);*/

        // Hold button A to aim and range limelight target
        

        m_driverController.leftTrigger()
                .whileTrue(Autos.LL_Left_Align(m_robotDrive, m_elevatorSubsystem, m_shooterSubsystem));

        m_driverController.rightTrigger()
                .whileTrue(Autos.LL_Right_Align(m_robotDrive, m_elevatorSubsystem, m_shooterSubsystem));

        // Hold button B to aim and range limelight target with minimum command
        m_driverController.b()
            .whileTrue(new RunCommand(
                ()-> m_robotDrive.drive(
                        m_robotDrive.limelight_aim_and_range()[1], 
                        m_robotDrive.limelight_aim_and_range()[0]*.1,
                        m_robotDrive.limelight_aim_and_range()[0], 
                        false), 
                m_robotDrive));


       // m_RightReefAlign, m_LeftReefAlign
       // m_driverController.rightTrigger()
          //  .whileTrue(m_RightReefAlign)
          //  .onFalse(new RunCommand(()->m_robotDrive.stop()));

      //  m_driverController.leftTrigger()
        //    .whileTrue(m_LeftReefAlign)
         //   .onFalse(new RunCommand(()->m_robotDrive.stop()));

        

        // -------------------------------- CORAL -----------------------------------------------
        // Left Trigger -> Run tube intake
        m_operatorController.rightTrigger(OIConstants.kTriggerButtonThreshold).
            whileTrue(m_shooterSubsystem.shootCoral());

        // Right Trigger -> Run tube intake in reverse
        m_operatorController.leftTrigger(OIConstants.kTriggerButtonThreshold).
            whileTrue(m_shooterSubsystem.intakeCoral());

        m_operatorController.start().
            whileTrue(m_shooterSubsystem.slowIntakeCoral());

        // A Button -> Elevator/Arm to level 1 position"
        //m_operatorController.a().onTrue(new RunCommand(()->{m_coralSubsystem.setElevatorLevel1();}));
       /* m_operatorController.a().whileTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel1();})
                            .andThen(()->{resetElevator();}));*/

        m_operatorController.a().whileTrue(new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(0.0,ControlType.kPosition))
                                .andThen(()->{resetElevator();}));
        //m_operatorController.a().whileTrue(startEnd(()->{m_elevatorSubsystem.elevatorClosedLoopController.setReference(0.0,ControlType.kPosition)}),()->resetElevator());
                                
        //.andThen(Commands.runOnce(()->{resetElevator();}), m_elevatorSubsystem))

        
        //m_operatorController.a().whileTrue(new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(52,ControlType.kPosition)));
        //m_operatorController.a().whileTrue(new RunCommand (()->{m_elevatorSubsystem.setElevatorLevel1();}));
        

        // X Button -> Elevator/Arm to level 2 position
        //m_operatorController.x().onTrue(new RunCommand(()->{m_coralSubsystem.setElevatorLevel2();}));
        //m_operatorController.x().whileTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel2();}));
        //m_operatorController.x().onTrue(new elevatorSetPointCommand(m_elevatorSubsystem, 35));
        m_operatorController.x().whileTrue(new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(18,ControlType.kPosition)));


//m_operatorController.a().onTrue(Commands.runOnce (()->{m_coralSubsystem.setElevatorLevel2();}));
//m_operatorController.x().onTrue(m_coralSubsystem.setElevatorLevel2());

        // B Button -> Elevator/Arm to level 3 position"
        //m_operatorController.b().onTrue(new RunCommand(()->{m_coralSubsystem.setElevatorLevel3();}));
        //m_operatorController.b().whileTrue(new RunCommand(()->{m_elevatorSubsystem.setElevatorLevel3();}));
        //m_operatorController.b().onTrue(new elevatorSetPointCommand(m_elevatorSubsystem, 91));
        m_operatorController.b().whileTrue(new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(46,ControlType.kPosition)));

        
        //Y Button -> Elevator/Arm to level 4 position
        m_operatorController.y().whileTrue(new RunCommand(()->m_elevatorSubsystem.elevatorClosedLoopController.setReference(90,ControlType.kPosition)));


        //m_operatorController.a().onTrue(Commands.runOnce (()->{m_coralSubsystem.setElevatorLevel3();}));

        // Y Button -> Run entry tube intake
        //m_operatorController.y().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel4));

        // -------------------------------- ALGAE -----------------------------------------------
        // Left Bumper -> Run ball intake, set to leave out when idle
        m_operatorController.leftBumper() 
            //  The returned trigger will be true when the axis value is greater than threshold.
             // Threshold is equivalent as joystick deadband in case the trigger is not completely released
            .whileTrue(m_algaeIntakeSubsystem.shootAlgae());
            //.whileFalse(m_algaeSubsystem.stopIntakeCommand());

        // Right Bumper -> Run ball intake in reverse, set to stow when idle
        m_operatorController.rightBumper()
            .whileTrue(m_algaeIntakeSubsystem.intakeAlgae());
            //.whileFalse(m_algaeSubsystem.stopIntakeCommand());

       /*  m_operatorController.povUp()
            .whileTrue(new RunCommand(
                ()->{m_armSubsystem.setArmLevelMax();})
            );*/

            m_operatorController.povUp()
            .whileTrue(new RunCommand(()->{m_armSubsystem.armController.setReference(-14.5,ControlType.kPosition);}));

            
            m_operatorController.povDown()
            .whileTrue(new RunCommand(()->{m_armSubsystem.armController.setReference(0,ControlType.kPosition);}));

            m_operatorController.povRight()
            .whileTrue(m_funnelSubsystem.runEntryOuttake());

            m_operatorController.povLeft()
            .whileTrue(m_funnelSubsystem.runEntryIntake());

    } // end configureBindings()

    // Add autonomous commands in autoChooser Combo Box dropdown list
    private void configureAutonomous() {
        // -------------------------------- USING 'CONVENTIONAL' METHOD TO EXECUTE AUTO COMMANDS ------------------------------------------------------------------------
        autoChooser.addOption("LL", Autos.LL_Mid_OneCoral(m_robotDrive, m_elevatorSubsystem, m_shooterSubsystem));
        autoChooser.addOption("LL2", Autos.LL_Side_TwoCoral(m_robotDrive, m_elevatorSubsystem, m_shooterSubsystem, m_funnelSubsystem));
        autoChooser.addOption("LL3", Autos.LL_Side_ThreeCoral(m_robotDrive, m_elevatorSubsystem, m_shooterSubsystem, m_funnelSubsystem));

        //Blues MJ commented some out
  /*      autoChooser.addOption("Blue Top 1 Coral Path Auto", new PathPlannerAuto("Blue Top 1 Coral Auto"));
       autoChooser.addOption("Blue Mid L4 1 Coral Auto", new PathPlannerAuto("Blue Mid L4 1 Coral Auto"));
       autoChooser.addOption("Blue Bottom 1 Coral Path Auto", new PathPlannerAuto("Blue Bottom 1 Coral Auto"));
      //  autoChooser.addOption("Blue Bottom 1 Coral Knock Auto", new PathPlannerAuto("Blue Bottom 1 Coral Knock Auto"));
      //  autoChooser.addOption("PID Blue Middle 1 Coral Auto", new PathPlannerAuto("PID Blue Middle 1 Coral Auto"));
       // autoChooser.addOption("Blue Middle L4 1 Coral Auto", new PathPlannerAuto("Blue Middle L4 1 Coral Auto"));
       autoChooser.addOption("Test Blue Mid L4 1 Coral Auto", new PathPlannerAuto("Test Blue Mid L4 1 Coral Auto"));


        //Reds MJ commented some out to rename
        autoChooser.addOption("Red Top 1 Coral Path Auto", new PathPlannerAuto("Red Top 1 Coral Auto"));
       // autoChooser.addOption("Red Mid 1 Coral Path Auto", new PathPlannerAuto("Red Middle 1 Coral Auto"));
        autoChooser.addOption("Red Bottom 1 Coral Path Auto", new PathPlannerAuto("Red Bottom 1 Coral Auto"));
        //autoChooser.addOption("Red Middle 1 Coral Knock Auto", new PathPlannerAuto("Red Middle 1 Coral Knock Auto"));
        autoChooser.addOption("Other Red Middle 1 Coral Knock Auto", new PathPlannerAuto("Copy of Red Middle 1 Coral Knock Auto"));*/
       // autoChooser.addOption("Copy of PID Red Middle 1 Coral Knock Auto", new PathPlannerAuto("Copy of PID Red Middle 1 Coral Knock Auto"));
  /* 
        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_robotDrive));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
*/
        // -------------------------------- USING 'PATHPLANNER' METHOD TO EXECUTE AUTO COMMANDS ------------------------------------------------------------------------
        //autoChooser.addOption("Drivetrain Practice (PathPlanner)", new DrivetrainPractice(this));
        SmartDashboard.putData("autonomous", autoChooser);
      
    } // end configureAutonomous()
    
/* 
    private void configureDashboard(){

        // Forward ports 5800, 5801, 5802, 5803, 5804, 5805, 5806, and 5807, 5808, 5809
        //   to enable Limelight communication while tethered to your robot over USB.
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
        // Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        // To add a second limelight, use a different set of local ports. 
        // Note the "+10" in the first PortForwarder.add() argument:
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port+10, "limelight-left.local", port);
        }

        //Create 'Driver' Tab
        //var driverTab = Shuffleboard.getTab("Driver");

        // Limelight is an USB camera, not HttpCamera
        // After Port Forwarding, you may access your Limelight's 
        // stream at http://roborio-5993-FRC.local:5800. 
//        driverTab.add(new edu.wpi.first.cscore.UsbCamera("limelight", "http://roborio-5993-frc.local:5800"));
        //UsbCamera cam1 = CameraServer.startAutomaticCapture();
        /*
        var limelightTable=NetworkTableInstance.getDefault().getTable("limelight");
        Shuffleboard.getTab("Vision").add("Limelight Stream",
        limelightTable.getEntry("stream"))
                .withWidget(BuiltInWidgets.kCameraStream);
        
    }
*/
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
/*   
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(.2, .2),    // (1,1)
            new Translation2d(.2, -.1)),    // (2, -1)
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(.3, 0, new Rotation2d(0)),   // (3,0,0)
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
*/
    //return m_chooser.getSelected();
    return autoChooser.getSelected();
  } // end getAutonomousCommand()

/*
  public Command ReadingRainbow() {
    m_ColorSensor.getColors(m_ColorSensor.getSensor().getRed(),
                            m_ColorSensor.getSensor().getBlue(),
                            m_ColorSensor.getSensor().getGreen(),
                            m_ColorSensor.getSensor().getProximity());
    System.out.print("Color = " + m_ColorSensor.getColor());

    if(m_ColorSensor.getColor().equals("Red")){
        //System.out.println(", running set X()");
       //return new RunCommand(() -> m_robotDrive.setX(), m_robotDrive);
        //return new InstantCommand(() -> m_robotDrive.setX(), m_robotDrive);
        return m_robotDrive.setXCommand();
    }
    else{
        //System.out.println(", running set H()");
        //return new RunCommand(() -> m_robotDrive.setH(), m_robotDrive);
        //return new InstantCommand(() -> m_robotDrive.setH(), m_robotDrive);
        return m_robotDrive.setHCommand();
        

    }

  }
*/

}   // end class RobotCOntainer
