// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double length = 26.5; //PC Specs
  public static final double width = 26.5; //PC Specs

  //Maybe need maxVolts later?

  //PID or feedforward? BOTH!

  public static final int backLeftSpeedMotor = 6; 
  public static final int backLeftAngleMotor = 5; 
  public static final int backRightSpeedMotor = 11; 
  public static final int backRightAngleMotor = 10; 
  public static final int frontLeftSpeedMotor = 9; 
  public static final int frontLeftAngleMotor = 8; 
  public static final int frontRightSpeedMotor = 13; 
  public static final int frontRightAngleMotor = 12;

  //NOTE: RENAME MOD NUMBERS TO POSITION ON BOT!!!!

  //module 1 constants: Front Left
  public static final int mod1DriveMotor = 9;
  public static final int mod1AngleMotor = 8;
  public static final int mod1CANCoder = 2;
  public static final double mod1AngleOffset = 324.93164;

  //module 2 constants: Front Right
  public static final int mod2DriveMotor = 13;
  public static final int mod2AngleMotor = 12;
  public static final int mod2CANCoder = 3;
  public static final double mod2AngleOffset = 318.422734375;

  //module 3 constants: Back Left
  public static final int mod3DriveMotor = 6;
  public static final int mod3AngleMotor = 5;
  public static final int mod3CANCoder = 1;
  public static final double mod3AngleOffset = 176.923828125;

  //module 4 constants: Back Right
  public static final int mod4DriveMotor = 11;
  public static final int mod4AngleMotor = 10;
  public static final int mod4CANCoder = 0;
  public static final double mod4AngleOffset = 67.939453125;

  //PID Constants
  public static final double speedkP = 0.05;
  public static final double speedkI = 0;
  public static final double speedkD = 0;

  public static final double anglekP = 0.05;
  public static final double anglekI = 0;
  public static final double anglekD = 0;


  //Gear Ratios (for the conversion factors)
  public static final double speedMotorRatio = 6.12;
  public static final double angleMotorRatio = 6.12;

  //Conversion Factors
  public static final double speedMotorPosFactor = Math.PI * speedMotorRatio;
  public static final double speedMotorVelFactor = speedMotorPosFactor / 60;
  public static final double angleMotorPosFactor = 360 / angleMotorRatio;
  //NEED ANGLE MOTOR GEAR RATIO

  
  //Max speed of the robot (in meters per second)
  public static final double maxSpeed = 4.5;
  //are you kidding me??????
  public static final double maxAngularVelocity = 11.5;
  //NEED!!!!!!


  //Lengths of the drivetrain, as well as the map of where the wheels are on the robot (like coordinates)
  public static final double robotLength = 26.5;
  public static final double robotWidth = 26.5;


  public static final SwerveDriveKinematics SwerveMap = new SwerveDriveKinematics(
    new Translation2d(robotLength / 2, robotWidth / 2), 
    new Translation2d(robotLength / 2, -robotWidth / 2),
    new Translation2d(-robotLength / 2, robotWidth / 2),
    new Translation2d(-robotLength / 2, -robotWidth /2)
    );





}

