// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //PC Specs: 26.5 in

  //Maybe need maxVolts later?

  //PID or feedforward? BOTH!

  

  //NOTE: RENAME MOD NUMBERS TO POSITION ON BOT!!!!

  //module 0 constants: Front Left                    //MG    //PC

  public static final int mod0DriveMotor = 59;        //59    //10
  public static final int mod0TurningMotor = 55;      //55    //11
  public static final int mod0CANCoder = 3;           //3   //2
  public static final Rotation2d mod0TurningOffset = Rotation2d.fromDegrees(148.2);

  //module 1 constants: Front Right module 3?
  public static final int mod1DriveMotor = 58;       //58    //9
  public static final int mod1TurningMotor = 54;       //54    //8
  public static final int mod1CANCoder = 0;       //0       //0
  public static final Rotation2d mod1TurningOffset = Rotation2d.fromDegrees(156.0);


  //module 2 constants: Back Left
  public static final int mod2DriveMotor = 53;        //53    //12
  public static final int mod2TurningMotor = 61;        //61    //13
  public static final int mod2CANCoder = 2;         //2       //1
  public static final Rotation2d mod2TurningOffset = Rotation2d.fromDegrees(166.1);


  //module 3 constants: Back Right
  public static final int mod3DriveMotor = 36;       //36    //9
  public static final int mod3TurningMotor = 52;       //52    //8
  public static final int mod3CANCoder = 1;       //1       //3
  public static final Rotation2d mod3TurningOffset = Rotation2d.fromDegrees(187.3);


  //PID Constants
  public static final double drivekP = 0.01; 
  public static final double drivekI = 0;
  public static final double drivekD = 0;

  public static final double turningkP = 0.0066; //init value 1
  public static final double turningkI = 0.0006; //init value 0.1
  public static final double turningkD = 0.0001; //init value 0


  //Gear Ratios (for the conversion factors)
  public static final double driveMotorRatio = 6.12;
  public static final double turningMotorRatio = 150 / 7;

  //Conversion Factors
  public static final double driveMotorPosFactor = Math.PI * driveMotorRatio;
  public static final double driveMotorVelFactor = driveMotorPosFactor / 60;
  public static final double turningMotorPosFactor = 360 / turningMotorRatio;

  
  //Max drive of the robot (in meters per second)
  public static final double maxSpeed = 4.5;
  //are you kidding me??????
  public static final double maxAngularVelocity = 7.5;
  //NEED!!!!!!

  


  //Lengths of the drivetrain, as well as the map of where the wheels are on the robot (like coordinates)
  public static final double robotLength = Units.inchesToMeters(23.875);
  public static final double robotWidth = Units.inchesToMeters(23.875);


  public static final SwerveDriveKinematics SwerveMap = new SwerveDriveKinematics(
    new Translation2d(robotLength / 2, robotWidth / 2), 
    new Translation2d(robotLength / 2, -robotWidth /2),
    new Translation2d(-robotLength / 2, robotWidth / 2),
    new Translation2d(-robotLength / 2, -robotWidth / 2)

    );





}

