// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FuelSubSystem;
import static frc.robot.Constants.FuelConstants.CENTER_DISTANCE_VELOCITY;
import static frc.robot.Constants.FuelConstants.FAR_DISTANCE_VELOCITY;
import static frc.robot.Constants.FuelConstants.FEEDER_LAUNCH_POWER;
import static frc.robot.Constants.FuelConstants.MID_DISTANCE_VELOCITY;
import static frc.robot.Constants.FuelConstants.NEAR_DISTANCE_VELOCITY;
import static frc.robot.Constants.FuelConstants.TEST_VELOCITY;

import org.opencv.features2d.Features2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // This is a Test Commit and Push

  // SubSystems
  private final DriveTrainSubsystem driveTrainSubsystem= new DriveTrainSubsystem();
  private final FuelSubSystem fuelSubSystem = new FuelSubSystem();
  private double rpmOffset = 10;

  // Chooser
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

      private final CommandXboxController operatorController =
      new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
      
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    // Start Position Commands - Not Used anymore
   
    // Launcher Commands
    
    //offseted shooting for auto
   
    // LimeLight distance Velocity for Auto
   
    // Feeder and Intake Commands
   
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


  }


public void setDesiredAngle() {
  driveTrainSubsystem.setDesiredAngle();
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Real Controls - Operator
    // Operator Intake form Ground LeftTrigger
    
    // Operator Left Bumber is Outtake
    
    // Operator Shooting is Right Trigger

    // Operator y starts Launcher distance Distance

    // Operator b is near shot

    // Operator a stops Launcher

    // Real Controls Driver
    // Default Command Arcade Style Drive

    // Climber controlled with Triggers


    // Stop Launcher Motors
    operatorController.a()
      .whileTrue(fuelSubSystem.stopLauncherCommand(fuelSubSystem));

    // Start Launcher Motors
    operatorController.x()
      .whileTrue(fuelSubSystem.launchVelocityCommand(fuelSubSystem, MID_DISTANCE_VELOCITY)); 
      //.whileTrue(fuelSubSystem.launchVelocityTestcommand(fuelSubSystem));
      // Not really RPM yet about 8:1 Ratio 550 -> 4400 RPM

    // Shoots Fuel 
    operatorController.rightBumper()
      .whileTrue(fuelSubSystem.setFeederCommand(fuelSubSystem, -0.8));

    // limelight distance shoot Trigger
    operatorController.leftBumper()
    .whileTrue(fuelSubSystem.launchVelocityLimelightCommand(fuelSubSystem));  

    // Intakes Fuel  
    operatorController.b()
      .whileTrue(fuelSubSystem.setIntakeCommand(fuelSubSystem, 0.8));

   // Outake Fuel
   operatorController.y()
      .whileTrue(fuelSubSystem.setIntakeCommand(fuelSubSystem, -0.8));

   //operatorController.povUp()
      //.onTrue(fuelSubSystem.changeTargetVelocityCommand(10));

   
   //operatorController.povDown()
      //.onTrue(fuelSubSystem.changeTargetVelocityCommand(-10)); 

    // Driver Controls
    // Climber Controls

    //fuelSubSystem.setDefaultCommand(fuelSubSystem.launchSpeedCommand(fuelSubSystem, 0));
    
  // Aiming Mode with Limelight
    m_driverController.leftBumper()
      .whileTrue(driveTrainSubsystem.aimCommand());

    // Fast Mode press and hold Driver Right Bumper
    m_driverController.rightBumper()
      .whileTrue(driveTrainSubsystem.driveArcade(
        () -> m_driverController.getLeftY() * OperatorConstants.FAST_DRIVE,
        () -> m_driverController.getRightX() * OperatorConstants.FAST_TURN));

    // DriveTrain Bindings
    driveTrainSubsystem.setDefaultCommand(
      driveTrainSubsystem.driveArcade(
        () -> m_driverController.getLeftY() * OperatorConstants.SLOW_DRIVE,
        () -> m_driverController.getRightX() * OperatorConstants.SLOW_TURN)
    );
        
  }

  // Autonomous Command
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new PathPlannerAuto("BackUpShoot");
    Command autoCommand = autoChooser.getSelected();
    PathPlannerAuto auto = (PathPlannerAuto) autoCommand;

    Pose2d startPose = auto.getStartingPose();
    driveTrainSubsystem.setStartPose(startPose);

    return autoChooser.getSelected();
    //return null;
  }

  
}
