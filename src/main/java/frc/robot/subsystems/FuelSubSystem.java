// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Locale.IsoCountryCode;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.FuelConstants.*;

public class FuelSubSystem extends SubsystemBase {

  // Fields for motor controllers and sensors related to the fuel system would be declared here
  private SparkMax feederRoller;
  //private SparkMax launcherLeft;
  //private SparkMax launcherRight;
  private SparkFlex intakeMotor;

  // Talon FX Controllers
  private TalonFX leftShooter;
  private TalonFX rightShooter;

  private final VelocityVoltage shooter_request = new VelocityVoltage(0).withSlot(0);

  //private SparkFlex indexMotor;
  // Test commit

  // Variable for Shooter Velocity Control
  private double launcherVelocitySet = 0;

  private NetworkTable limTable;
  private NetworkTableEntry tx; //yw ofc :)
  private NetworkTableEntry ta;

  /** Creates a new FuelSubSystem. */
  public FuelSubSystem() {

      // Initialize motor controllers and sensors here
      feederRoller = new SparkMax(Constants.FuelConstants.FUEL_FEEDER_ID, MotorType.kBrushless);
      intakeMotor = new SparkFlex(Constants.FuelConstants.FUEL_INTAKE_ID, MotorType.kBrushless);

      // Setup Shooter TalonFX Motors
      leftShooter = new TalonFX(Constants.FuelConstants.FUEL_SHOOTER_LEFT_ID);
      leftShooter.setNeutralMode(NeutralModeValue.Coast);

      rightShooter = new TalonFX(Constants.FuelConstants.FUEL_SHOOTER_RIGHT_ID);
      rightShooter.setNeutralMode(NeutralModeValue.Coast);
      
      // Configuration for TalonFX Motors
      MotorOutputConfigs rightShooterConfigs = new MotorOutputConfigs();
      rightShooterConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
      rightShooter.getConfigurator().apply(rightShooterConfigs);

      MotorOutputConfigs leftShooterConfigs = new MotorOutputConfigs();
      leftShooterConfigs.Inverted = InvertedValue.Clockwise_Positive;
      leftShooter.getConfigurator().apply(leftShooterConfigs);

      var slot0ConfigsFlywheel = new Slot0Configs();
      slot0ConfigsFlywheel.kS = 0.1;
      slot0ConfigsFlywheel.kV = 0.12;
      slot0ConfigsFlywheel.kP = 0.11;
      slot0ConfigsFlywheel.kI = 0;
      slot0ConfigsFlywheel.kD = 0;
      rightShooter.getConfigurator().apply(slot0ConfigsFlywheel);
      leftShooter.getConfigurator().apply(slot0ConfigsFlywheel);

     
      // Setup Configuation for Intake and Feeder Motors
      SparkMaxConfig feederConfig = new SparkMaxConfig();
      feederConfig.smartCurrentLimit(Constants.FuelConstants.CURRENT_LIMIT);
      feederConfig.idleMode(IdleMode.kBrake);
      feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intakeMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //indexMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     

      // Smart Dashboard
      double rightShooterVelocity = rightShooter.getVelocity().getValueAsDouble()*60;
      double leftShooterVelocity = leftShooter.getVelocity().getValueAsDouble()*60;

      SmartDashboard.putNumber("Left Launcher RPM", leftShooterVelocity);
      SmartDashboard.putNumber("Right Launcher RPM:", rightShooterVelocity);
      SmartDashboard.putNumber("Left Launch Amps", 0);
      SmartDashboard.putNumber("Right Launch Amps", 0);
      SmartDashboard.putNumber("Shoot Velocity Set", 0);

      // binding limelight
      limTable = NetworkTableInstance.getDefault().getTable("limelight");
      tx = limTable.getEntry("tx");
      ta = limTable.getEntry("ta");
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the Velocities of the Launcher
    double rightShooterVelocity = rightShooter.getVelocity().getValueAsDouble()*60;
    double leftShooterVelocity = leftShooter.getVelocity().getValueAsDouble()*60;
    SmartDashboard.putNumber("Left Launcher RPM", leftShooterVelocity);
    SmartDashboard.putNumber("Right Launcher RPM:", rightShooterVelocity);    

  }

  // Methods

  // Simple Turn on Left Launcher - will right follow?
  public void setLaunchPower(double power) {
    rightShooter.set(power); 
    leftShooter.set(power);
  }

  // Modified for TalonFX
  public void setLaunchVelocity(double velocity) {
    rightShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
    leftShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
    
  }

  public void setLaunchVelocityFromSetPoint() {
    //double velocity = SmartDashboard.getNumber("Shoot Velocity Set", 0);
    //leftLaunchClosedLoopController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    //rightLaunchClosedLoopController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

    public void setLaunchVelocityFromLimelight() {
    double tagArea = ta.getDouble(0.9); //0.75
    double velocity = -672.98*Math.pow(tagArea, 3)+ 1948.8*Math.pow(tagArea, 2)-1872.5*tagArea+1246.2;
    velocity = velocity;
    rightShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
    leftShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
    launcherVelocitySet =  velocity;
    SmartDashboard.putNumber("Shoot Velocity Set", launcherVelocitySet);
    
  }



  // Stop Launcher
  public void stopLauncher() {
    rightShooter.set(0); 
    leftShooter.set(0);
  }
  public Command autoStartLauncher() {

      return Commands.run(() -> feederRoller.setVoltage(MID_DISTANCE_VELOCITY), 
                           this);
  }
  public Command autoIntake() {

      return Commands.run(() -> 
      
      new ParallelCommandGroup(
      autoStartLauncher(),
      feederSpeedCommand(this, 0.8)              
      )
      
      );
  }


  public void setFeederPower(double power) {
    feederRoller.set(power);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public void setIntakeFeederPower(double power) {
    intakeMotor.set(power);
    feederRoller.set(-power);
    //indexMotor.set(power);
  }

  public void setFeederLaunchPower(double power) {
    intakeMotor.set(-power);
    feederRoller.set(-power);
  }

  
   public void setFeederSpeed(double power) {
    feederRoller.set(-power);
  }

  public void changeTargetVelocity(double delta) {
    launcherVelocitySet += delta;
    if (launcherVelocitySet < 0) {
      launcherVelocitySet = 0;
    }
    SmartDashboard.putNumber("Shoot Velocity Set", launcherVelocitySet);
  }


  // Command Factories

  // Test Commands to turn on and off the Launch Motors
  // launchSpeeedCommand net used
  public Command launchSpeedCommand(FuelSubSystem fuelSubSystem, double speed) {
    return Commands.runEnd(() -> setLaunchPower(speed), () -> setLaunchPower(0), fuelSubSystem);
  }

  public Command feederSpeedCommand(FuelSubSystem fuelSubsystem, double speed) {
    return Commands.run(() -> setFeederSpeed(speed));
  }

  // Use this oune
  public Command launchVelocityCommand(FuelSubSystem fuelSubSystem, double velocity) {
    return Commands.run(() -> setLaunchVelocity(velocity));
  }

  // Temporary Command for testing velocity
  public Command launchVelocityTestcommand(FuelSubSystem fuelSubSystem) {
    return Commands.run(() -> setLaunchVelocityFromSetPoint());
  }

  // Distance Test Command
  public Command launchVelocityLimelightCommand(FuelSubSystem fuelSubSystem) {
    return Commands.run(() -> setLaunchVelocityFromLimelight());
  }

  public Command stopLauncherCommand(FuelSubSystem fuelSubSystem) {
    return Commands.run(() -> stopLauncher());
  }
  public Command autoStartLauncherLeft(double velocity) {

      return Commands.runEnd(() -> setLaunchVelocity(velocity), () -> setLaunchPower(0));
      
  }
  public Command autoStartLauncherRight(double velocity) {

      return Commands.runEnd(() -> setLaunchVelocity(velocity), () -> setLaunchPower(0)); //this could be wrong hopefully not yay
      
  }

  public Command intakeSpeedCommand(FuelSubSystem fuelSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(() -> setIntakeFeederPower(forward.getAsDouble() - reverse.getAsDouble()), fuelSubsystem);
  }
  public Command setFeederCommand(FuelSubSystem fuelSubSystem, double speed) {
    return Commands.runEnd(() -> setFeederLaunchPower(speed), () -> setFeederLaunchPower(0));
  }

  public Command setIntakeCommand(FuelSubSystem fuelSubSystem, double speed) {
    return Commands.runEnd(() -> setIntakeFeederPower(speed), () -> setIntakeFeederPower(0));
  }

  public Command changeTargetVelocityCommand(double delta) {
    return Commands.runOnce(() -> changeTargetVelocity(delta));
  }

}
