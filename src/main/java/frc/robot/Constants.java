// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 10;
    public static final int LEFT_FOLLOWER_ID = 11;
    public static final int RIGHT_LEADER_ID = 12;
    public static final int RIGHT_FOLLOWER_ID = 13;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // For Localization - units in meteres
    public static final double TRACK_WIDTH = 0.5334;
    public static final double WHEEL_RADIUS = 0.0762;
    public static final int ENCODER_RESOLUTION = 42;
    public static final double GEAR_RATIO = 8.46;
    public static final double DISTANCE_ADJUST = 0.66;

    // For gyroscope
    public static final double GYRO_OFFSET = 180.0;
  }

  public static final class FuelConstants {
    // Motor controller IDs for fuel system motors
    public static final int FUEL_INTAKE_ID = 23;
    public static final int FUEL_FEEDER_ID = 20;
    public static final int FUEL_SHOOTER_LEFT_ID = 21;
    public static final int FUEL_SHOOTER_RIGHT_ID = 22;
    public static final int FUEL_HOPPER_ID = 24; // extendo of the Hopper

    public static final int CLIMBER_ID = 30;

    // Current limit for fuel system motors. 40A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging motors
    public static final int CURRENT_LIMIT = 60;

    public static final int LAUNCHER_CURRENT_LIMIT = 60;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10; //from kitbotA might need to change

    public static final double FEEDER_LAUNCH_POWER = 0.8;
    public static final double FEEDER_STIR_POWER = -0.8;

    public static final double NEAR_DISTANCE_VELOCITY  = 650;
    public static final double CENTER_DISTANCE_VELOCITY  = 700;
    public static final double MID_DISTANCE_VELOCITY = 750;
    public static final double FAR_DISTANCE_VELOCITY = 800;

    public static final double LEFT_AUTO_SHOOTER_OFFSET = 50;
    public static final double CENTER_AUTO_CLIMB_SHOOTER_OFFSET = 90;
    public static final double RIGHT_AUTO_SHOOTER_OFFSET = 50;
    public static final double CENTER_AUTO_SHOOTER_OFFSET = 30;

    public static final double TEST_VELOCITY = 500;
  }

  public static class ClimberConstants {
    public static final int REST_POSITION = 0;
    public static final int SHOOTING_POSITION = 250;  //climber down
  }

  public static class OperatorConstants {
        // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .4;

    // Slow and Fast Mode Constants
    public static final double SLOW_DRIVE = 0.5;
    public static final double SLOW_TURN = 0.4;
    public static final double FAST_DRIVE = 0.9;
    public static final double FAST_TURN = 0.5;
  }
}
