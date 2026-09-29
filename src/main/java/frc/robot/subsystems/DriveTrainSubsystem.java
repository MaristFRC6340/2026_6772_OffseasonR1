// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.SPI;




public class DriveTrainSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;



  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final DifferentialDrive drive;

  // Using Old Gyroscope for Testing Angles
  // Creating and Binding to Port
  ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private NetworkTable limTable;
  private NetworkTableEntry tx; //yw ofc :)
  private NetworkTableEntry ta;

  // Defining the Pose Estimator
  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final DifferentialDriveKinematics m_kinematics;

  private Field2d field = new Field2d();

  private double turnError;
  private double kP = 0.015;
  private double turnPower;
  private double gearMultiply = 1;

  // For heading control
  private double desiredAngle = 0;
  private double kPA = 0.1;
  private double driveTurnError = 0;

  // Preset Poses for Autonomous
  private Pose2d centerStart = new Pose2d(3.75, 4, new Rotation2d(180));
  private Pose2d leftStart = new Pose2d(3.75, 5.31, new Rotation2d(180));
  private Pose2d rightStart = new Pose2d(3.75, 2.89, new Rotation2d(180));

public DriveTrainSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);


    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);

    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


     drive = new DifferentialDrive(leftLeader, rightLeader);

     m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    // Reset Encoders
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    // Reset Gyroscope
    m_gyro.reset();

    // Start Poses
    // Center: 3.75, 4, 180
    // Left: 3.75, 5.31

    m_PoseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      -m_leftEncoder.getPosition()  * 2 * Math.PI * WHEEL_RADIUS / (GEAR_RATIO*gearMultiply),
      -m_rightEncoder.getPosition()  * 2 * Math.PI * WHEEL_RADIUS / (GEAR_RATIO*gearMultiply),

      new Pose2d(3.5, 4, new Rotation2d(0)));
      //center

      //new Pose2d(3.5, 5.31, new Rotation2d(0)));
      //left

      //new Pose2d(3.5, 2.88, new Rotation2d(0)));
      //right
      

      SmartDashboard.putData("Field", field);
      SmartDashboard.putString("Pose2D", m_PoseEstimator.getEstimatedPosition().toString());
      SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
      SmartDashboard.putNumber("Left Distance", 0);
      SmartDashboard.putNumber("Right Distance", 0);
      SmartDashboard.putNumber("Forward Input", 0);
      SmartDashboard.putNumber("Turn Input", 0);
      SmartDashboard.putNumber("Turn Power Adjust", 0);
      SmartDashboard.putNumber("Desired Angle", 0);

      // binding limelight
      limTable = NetworkTableInstance.getDefault().getTable("limelight");
      tx = limTable.getEntry("tx");
      ta = limTable.getEntry("ta");

      // Turning off all Path Planner for Now
       
      RobotConfig config2;
    try {
      config2 = RobotConfig.fromGUISettings();
    }
    catch (Exception e) {
      config2 = null;
      e.printStackTrace();
    }

    //configure autobuilder
    AutoBuilder.configure(
      
      this::getPose,
      this::resetPose,
      this::getSpeeds, 
      (speeds, feedforwards) -> driveSlow(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond), // Need to fix: method that takes speed and speed of rotation
      //new PPLTVController(0.02),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.2, 0.0, 0.0) // Rotation PID constants
            ),
      config2, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          //return alliance.get() == DriverStation.Alliance.Red;
          return false;
        }
        return false;
      },
      this
    );
    
    // Line Up Desired Angle
    desiredAngle = -1 * (m_gyro.getAngle() + GYRO_OFFSET);
     
}



  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    double leftDistance = -m_leftEncoder.getPosition()  * 2 * Math.PI * WHEEL_RADIUS / (GEAR_RATIO*gearMultiply);
    double rightDistance = -m_rightEncoder.getPosition()  * 2 * Math.PI * WHEEL_RADIUS / (GEAR_RATIO*gearMultiply) ;

    // Reading Gyroscope and publishing to Network Table
    double heading = -1 * (m_gyro.getAngle() + GYRO_OFFSET); // Adjust
    SmartDashboard.putNumber("Heading", heading);

    Rotation2d headingRotation2d = new Rotation2d(Math.toRadians(heading));



    // Update Pose Estimate
    m_PoseEstimator.update(
    //m_gyro.getRotation2d(), 
    headingRotation2d,
    leftDistance,
    rightDistance);

    SmartDashboard.putString("Pose2D", m_PoseEstimator.getEstimatedPosition().toString());

    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Distance", leftDistance);
    SmartDashboard.putNumber("Right Distance", rightDistance);

    // Publish for AdvantageScope
    field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);
  }

  // Set Desired Angle
  public void setDesiredAngle() {
    desiredAngle = -1 * (m_gyro.getAngle() + GYRO_OFFSET);
    SmartDashboard.putNumber("Desired Angle", desiredAngle);
  }

    public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }


  public void resetPose(Pose2d pos) {
    
    double angle = pos.getRotation().getDegrees();
    double x = pos.getX();
    double y = pos.getY();


    m_PoseEstimator.update(
    new Rotation2d(angle), x,y);
    }

  // For Path Planner and AdvantageScope
  // Now it Works!
  public void setStartPose(Pose2d startPose) {
    m_PoseEstimator.resetTranslation(new Translation2d(startPose.getX(), startPose.getY()));
  }

  public ChassisSpeeds getSpeeds() {
 
    double leftVelocity = m_leftEncoder.getVelocity() * 2 * Math.PI * WHEEL_RADIUS / (GEAR_RATIO*gearMultiply);
    double rightVelocity = m_leftEncoder.getVelocity() * 2 * Math.PI * WHEEL_RADIUS / (GEAR_RATIO*gearMultiply);

    var wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    return chassisSpeeds;
  }

  public void driveAim() {
    turnError = tx.getDouble(0);
    turnPower = kP * turnError;
    drive.arcadeDrive(0, turnPower);
    desiredAngle = -1 * (m_gyro.getAngle() + GYRO_OFFSET);
    SmartDashboard.putNumber("Desired Angle", desiredAngle);
  }

   public void driveSlow(double x, double z) {

    
      // Adjust x and z to account for speed:
      x = x/1;
      z = z/1;
      //z = z / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO / ENCODER_RESOLUTION);
      //the golden dandilion is a golden dandilion - canman 2026
      SmartDashboard.putNumber("Forward Input", x);
      SmartDashboard.putNumber("Turn Input", z);
      if (x > 0.5) {
        x = 0.5;
      }
      if (x < -0.5) {
        x = -0.5;
      }
      if (z > 0.4) {
        z = 0.4;
      }
      if (z < -0.4) {
        z = -0.4;
      }
      drive.arcadeDrive(-x, -z);
      //arcadeDriveV2(-x, -z);
      setDesiredAngle();
  }

  // Uses Gyroscope to keep robot straight
  // This is now the default Teleoperation Mode
  public void arcadeDriveV2(double xSpeed, double zRotation) {
    // Regular Drive
    if (Math.abs(zRotation) > 0.1) {
        drive.arcadeDrive(xSpeed, zRotation);
        desiredAngle = -1 * (m_gyro.getAngle() + GYRO_OFFSET);
        SmartDashboard.putNumber("Desired Angle", desiredAngle);
    }
    // PID Heading Control Mode when not turning
    else {
      driveTurnError = (-1 * (m_gyro.getAngle() + GYRO_OFFSET)) - desiredAngle;
      double adjustPower = driveTurnError * kPA;
      if (adjustPower > 0.2) {
        adjustPower = 0.2;
      }
      if (adjustPower < -0.2) {
        adjustPower = -0.2;
      }
      SmartDashboard.putNumber("Turn Power Adjust", adjustPower);
      drive.arcadeDrive(xSpeed, adjustPower);
    }
    

  }

  // For setting Auto Start Positions
  public Command setCenterPose() {
    return this.runOnce(() -> resetPose(centerStart));
  }

  public Command setLeftPose() {
    return this.runOnce(() -> resetPose(leftStart));
  }

  public Command setRightPose() {
    return this.runOnce(() -> resetPose(rightStart));
  }


  // Modified to use arcadeDriveV2 - a PID direction controlled method 
  // To keep Robot driving straight
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        //() -> drive.arcadeDrive(xSpeed.getAsDouble() * driveSpeed, zRotation.getAsDouble() * turnSpeed));
        () -> arcadeDriveV2(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
  
  
  /* 
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> driveSubCommand(xSpeed.getAsDouble() * driveSpeed, zRotation.getAsDouble() * turnSpeed));
  }
        */

  public Command aimCommand() {    
    return this.run(
      () -> driveAim());
  }
}
