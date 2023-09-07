// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public SwerveModule[] dt;
  

  //dt is DriveTrain
  public SwerveDrive() {
    //creates a "map" of the robot, recording the position of each swerve wheel relative to the others
    dt = new SwerveModule[] 
    {
      new SwerveModule(0, Constants.mod0DriveMotor, Constants.mod0AngleMotor, Constants.mod0CANCoder, Constants.mod0AngleOffset),
      new SwerveModule(1, Constants.mod1DriveMotor, Constants.mod1AngleMotor, Constants.mod1CANCoder, Constants.mod1AngleOffset),
      new SwerveModule(2, Constants.mod2DriveMotor, Constants.mod2AngleMotor, Constants.mod2CANCoder, Constants.mod2AngleOffset),
      new SwerveModule(3, Constants.mod3DriveMotor, Constants.mod3AngleMotor, Constants.mod3CANCoder, Constants.mod3AngleOffset)
    };

      Timer.delay(1);
      //resetToAbsolute2();
    

  }


  public void drive(Translation2d translation, double rotation, boolean isAuto) {

    SwerveModuleState[] swerveModuleStates = Constants.SwerveMap.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

    for(SwerveModule module : dt) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isAuto);
    }

  }

  public void resetToAbsolute2(){
    for(SwerveModule mod : dt){
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
