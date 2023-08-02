// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private SwerveModule[] dt;
  

  //dt is DriveTrain
  public SwerveDrive() {
    //creates a "map" of the robot, recording the position of each swerve wheel relative to the others
    dt = new SwerveModule[] 
    {
      new SwerveModule(1, Constants.mod1DriveMotor, Constants.mod1AngleMotor, Constants.mod1CANCoder, Constants.mod1AngleOffset),
      new SwerveModule(2, Constants.mod2DriveMotor, Constants.mod2AngleMotor, Constants.mod2CANCoder, Constants.mod2AngleOffset),
      new SwerveModule(3, Constants.mod3DriveMotor, Constants.mod3AngleMotor, Constants.mod3CANCoder, Constants.mod3AngleOffset),
      new SwerveModule(4, Constants.mod4DriveMotor, Constants.mod4AngleMotor, Constants.mod4CANCoder, Constants.mod4AngleOffset)
    };

  }


  public void drive(Translation2d translation, double rotation, boolean isAuto) {

    SwerveModuleState[] swerveModuleStates = Constants.SwerveMap.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

    for(SwerveModule module : dt) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isAuto);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
