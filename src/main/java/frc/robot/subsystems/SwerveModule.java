// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.SwerveOpt;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoder;


/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;



    //Establishing the motors (with their built-in encoders), CANCoder, and PID loops here
    CANSparkMax speedMotor;
    CANSparkMax angleMotor;

    RelativeEncoder speedEncoder;
    RelativeEncoder angleEncoder;
    CANCoder bestEncoder;

    SparkMaxPIDController speedPID;
    SparkMaxPIDController anglePID;

    //feedforward loop here: will use for auto


    //Returns the current angle of the wheel
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public double getAngleVoltage() {
        return angleMotor.getBusVoltage();
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
    public void resetToAbsolute() {
        System.out.print(getCANCoder().getDegrees());
        double absolutePosition =  getCANCoder().getDegrees() - angleOffset.getDegrees();
        angleEncoder.setPosition(absolutePosition);
      }

      //try setting to absolute position?


    
      //the skeleton of each individual swerve module
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int CANCoderID, Rotation2d OffsetAngle) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = OffsetAngle;


        //Creating the stuff established above and configuring them       
        bestEncoder = new CANCoder(CANCoderID);

        
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();
        //anglePID = new PIDController(Constants.anglekP, Constants.anglekI, Constants.anglekD);
        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setInverted(false);
        angleEncoder.setPositionConversionFactor(Constants.angleMotorPosFactor);
        angleMotor.setSmartCurrentLimit(20);
        angleMotor.burnFlash();
        angleMotor.enableVoltageCompensation(12);
        //anglePID.enableContinuousInput(-180, 180);
        anglePID.setP(Constants.anglekP);
        anglePID.setI(Constants.anglekI);
        anglePID.setD(Constants.anglekD);
        anglePID.setFF(0);
        angleEncoder.setPosition(0.0);
        resetToAbsolute();

        speedMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        speedEncoder = speedMotor.getEncoder();
        speedPID = speedMotor.getPIDController();
        //speedPID = new PIDController(Constants.speedkP, Constants.speedkI, Constants.speedkD);
        speedMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setInverted(false);
        speedEncoder.setPositionConversionFactor(Constants.speedMotorPosFactor);
        speedEncoder.setVelocityConversionFactor(Constants.speedMotorVelFactor);
        speedEncoder.setPosition(0);
        speedMotor.setSmartCurrentLimit(80);
        speedMotor.burnFlash();
        speedMotor.enableVoltageCompensation(12);
        speedEncoder.setPosition(0.0);
        speedPID.setP(Constants.speedkP);
        speedPID.setI(Constants.speedkI);
        speedPID.setD(Constants.speedkD);
        speedPID.setFF(0);


        


        //gets the last known angle of the SwerveModuleState
        lastAngle = getState().angle;
    }
    //calculates the desired position of the swerve wheel
    public void setDesiredState(SwerveModuleState desiredState, boolean isAuto) {
        desiredState = SwerveOpt.optimize(desiredState, getState().angle);

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

        anglePID.setReference(angle.getDegrees(), ControlType.kPosition);
        //double angleMotorValue = anglePID.calculate(lastAngle.getDegrees(), angle.getDegrees());
        //angleMotor.set(angleMotorValue);
        lastAngle = angle;
    }



}
