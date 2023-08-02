// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoder;


/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;

    //set offsets
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;



    //Establishing the motors (with their built-in encoders), CANCoder, and PID loops here
    CANSparkMax speedMotor;
    CANSparkMax angleMotor;

    RelativeEncoder speedEncoder;
    RelativeEncoder angleEncoder;
    CANCoder bestEncoder;

    PIDController speedPID;
    PIDController anglePID;

    //feedforward loop here


    //Returns the current angle of the wheel
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }



    //depicts a new SwerveModuleState using the current velocity and angle of the wheel
    public SwerveModuleState getState() {
        return new SwerveModuleState(speedEncoder.getVelocity(), getAngle());
    }

    //gets the position of the CANCoders
    public Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(bestEncoder.getAbsolutePosition());
    }


    //Resets the position of the CANCoders to their default state
    private void resetToAbsolute() {
        double absolutePosition = getCANCoder().getDegrees() - angleOffset.getDegrees();
        bestEncoder.setPosition(absolutePosition);
      }
      //the skeleton of each individual swerve module
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int CANCoderID, double angleOffset) {
        this.moduleNumber = moduleNumber;


        //Creating the stuff established above and configuring them       
        bestEncoder = new CANCoder(CANCoderID);

        speedMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        speedEncoder = speedMotor.getEncoder();
        speedPID = new PIDController(Constants.speedkP, Constants.speedkI, Constants.speedkD);
        speedMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setInverted(false);
        speedEncoder.setPositionConversionFactor(Constants.speedMotorPosFactor);
        speedEncoder.setVelocityConversionFactor(Constants.speedMotorVelFactor);
        speedEncoder.setPosition(0);
        speedPID.enableContinuousInput(-180, 180);

        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = new PIDController(Constants.anglekP, Constants.anglekI, Constants.anglekD);
        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setInverted(false);
        angleEncoder.setPositionConversionFactor(Constants.angleMotorPosFactor);
        resetToAbsolute();


        //gets the last known angle of the SwerveModuleState
        lastAngle = getState().angle;
    }
    //calculates the desired position of the swerve wheel
    public void setDesiredState(SwerveModuleState desiredState, boolean isAuto) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        setSpeed(desiredState, isAuto);
        setAngle(desiredState);
    }
    //calculate the necessary speed for the speed motor and set the motor to that speed
    private void setSpeed(SwerveModuleState desiredState, boolean isAuto) {
        if (isAuto == false) {
            double speedMotorOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
            speedMotor.set(speedMotorOutput);
        }
        else {
            //auto stuff with the feedforward loop; do it later:D
        }

    
    }
    //runs the PID loop for the angle motor, but only if the required speed is greater than 1% motor power, then powers the angle motor
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

        double angleMotorOutput = anglePID.calculate(lastAngle.getDegrees(), angle.getDegrees());
        angleMotor.set(angleMotorOutput);
        lastAngle = angle;
    }



}
