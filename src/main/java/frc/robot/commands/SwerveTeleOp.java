// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;

public class SwerveTeleOp extends CommandBase {



  private SwerveDrive driveSwerve;



  private Double translationSupply;
  private Double rotationSupply;
  private Double strafeSupply;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);

  /** Creates a new SwerveTeleOp. */
  public SwerveTeleOp(SwerveDrive driveSwerve, Double translationSupply, Double rotationSupply, Double strafeSupply) {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(driveSwerve);
  this.driveSwerve = driveSwerve;
  this.translationSupply = translationSupply;
  this.rotationSupply = rotationSupply;
  this.strafeSupply = strafeSupply;

  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = translationLimiter.calculate(MathUtil.applyDeadband(Robot.m_robotContainer.driverL.getY(), 0.1));
    double rotationValue = rotationLimiter.calculate(MathUtil.applyDeadband(Robot.m_robotContainer.driverR.getX(), 0.1));
    double strafeValue = strafeLimiter.calculate(MathUtil.applyDeadband(Robot.m_robotContainer.driverL.getX(), 0.1));

    driveSwerve.drive(new Translation2d(translationValue, strafeValue).times(Constants.maxSpeed), 
    rotationValue * Constants.maxAngularVelocity, 
    true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
