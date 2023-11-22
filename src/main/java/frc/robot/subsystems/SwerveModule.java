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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.SwerveOpt;
import frc.robot.CANSparkMaxUtil;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.CANSparkMaxUtil.Usage;

import java.io.Console;

import com.ctre.phoenix.sensors.CANCoder;


/** Add your docs here. */
public class SwerveModule {
    public final int moduleNumber;

    public final Rotation2d turningOffset;



    //Establishing the motors (with their built-in encoders), CANCoder, and PID loops here
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    public final RelativeEncoder turningEncoder;

    public final CANCoder bestTurningEncoder;

    //SparkMaxPIDController speedPID;
    //SparkMaxPIDController turningPID;

    private final PIDController drivePID; 
    private final PIDController turningPID;

    //feedforward loop here: will use for auto

      //the skeleton of each individual swerve module
    public SwerveModule(int moduleNumber, int driveMotorId, int turningMotorId, int canCoderId, Rotation2d turningOffest) {
        this.moduleNumber = moduleNumber;
        this.turningOffset = turningOffest;

        this.bestTurningEncoder = new CANCoder(canCoderId);

        this.turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        this.turningMotor.restoreFactoryDefaults();
        this.turningMotor.setIdleMode(IdleMode.kBrake);
        this.turningMotor.setInverted(true);
        this.turningMotor.setSmartCurrentLimit(20);
        this.turningMotor.enableVoltageCompensation(12);
        this.turningMotor.burnFlash();
    
    
        this.turningEncoder = this.turningMotor.getEncoder();
        this.turningEncoder.setPositionConversionFactor(Constants.turningMotorPosFactor);
        // TODO is setVelocityConversionFactor needed here
        //turningPID = turningMotor.getPIDController();

        this.turningPID = new PIDController(Constants.turningkP, Constants.turningkI, Constants.turningkD);
        // this.turningPID.enableContinuousInput(-180, 180);
        
        CANSparkMaxUtil.setCANSparkMaxBusUsage(this.turningMotor, Usage.kPositionOnly);
        
        //turningPID.setP(Constants.turningkP);
        //turningPID.setI(Constants.turningkI);
        //turningPID.setD(Constants.turningkD);
        //turningPID.setFF(0);
        //resetToAbsolute();

        this.driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        this.driveMotor.restoreFactoryDefaults();
        this.driveMotor.setSmartCurrentLimit(80);
        this.driveMotor.enableVoltageCompensation(12);
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.driveMotor.setInverted(false);
        this.driveMotor.burnFlash();


        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(Constants.driveMotorPosFactor);
        this.driveEncoder.setVelocityConversionFactor(Constants.driveMotorVelFactor);
        this.driveEncoder.setPosition(0);

        this.drivePID = new PIDController(Constants.drivekP, Constants.drivekI, Constants.drivekD);

    }
    

    /**
     * Returns the current turning turning of the wheel
     * @return The turning in degrees.
     */
    // public Rotation2d getTurningAngle() {
    //     return Rotation2d.fromDegrees(this.turningEncoder.getPosition());
    // }

    /**
     * Returns the current voltage used by the turning motor
     * @return The voltage.
     */
    public double getTurningMotorVoltage() {
        return this.turningMotor.getBusVoltage();
    }

    /**
     * Depicts a new SwerveModuleState using the current velocity and turning of the wheel
     * @return
     */
    public SwerveModuleState getState() {
        //return new SwerveModuleState(this.driveEncoder.getVelocity(), getTurningAngle());
        return new SwerveModuleState(this.driveEncoder.getVelocity(), this.getCANCoder());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }
    

    /**
     * Return the postion of the better CAN encoder.
     * @return Current position in degrees.
     */
    public Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(this.bestTurningEncoder.getAbsolutePosition());
    }

    //Resets the position of the CANCoders to their default state
    /**
     * 
     */
    public void resetToAbsolute() {
        System.out.print(getCANCoder().getDegrees());
        // double absolutePosition =  getCANCoder().getDegrees() - turningOffset.getDegrees();
        this.turningEncoder.setPosition(turningOffset.getDegrees());
      }

      //try setting to absolute position?


    

    //calculates the desired position of the swerve wheel
    public void setDesiredState(SwerveModuleState desiredState, boolean isAuto) {
        //Rotation2d encoderRotation = new Rotation2d(turningEncoder.getPosition());
        //desiredState = SwerveOpt.optimize(desiredState, getState().angle);
        //desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

        setAngle(desiredState);
        setSpeed(desiredState, isAuto);
    }
    //calculate the necessary speed for the speed motor and set the motor to that speed
    private void setSpeed(SwerveModuleState desiredState, boolean isAuto) {
        if (isAuto == false) {
            double driveMotorOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
            driveMotor.set(driveMotorOutput);
        }
        else {
            //auto stuff with the feedforward loop; do it later:D
        }

    
    }
    //runs the PID loop for the turning motor, but only if the required speed is greater than 1% motor power, then powers the turning motor
    private void setAngle(SwerveModuleState desiredState) {
        SwerveModuleState currentState = this.getState();
        // Rotation2d desiredTurningAngle =
        // (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01))
        //   ? currentState.angle
        //   : desiredState.angle;

        //turningPID.setReference(turning.getDegrees(), ControlType.kPosition);
        double currentDegrees = (currentState.angle.getDegrees() - this.turningOffset.getDegrees());
        currentDegrees = currentDegrees < -180 ? currentDegrees + 360 : currentDegrees;
        currentDegrees = currentDegrees > 180 ? currentDegrees - 360 : currentDegrees;
        double desiredDegrees = desiredState.angle.getDegrees() % 360;
        double diff = (currentDegrees - desiredDegrees + 180) % 360 - 180;
        diff = diff < -180 ? diff + 360 : diff;
        diff = diff > 180 ? diff - 360 : diff;
        // currentDegrees = -170
        // desiredDegrees = -179
        // off by +9


        // off by 11
        // desiredDegrees = 179


        // 190
        // 430
        //desiredState = SwerveOpt.optimize(desiredState, getState().angle);
        
        
        double turningMotorValue = Math.abs(diff) < 1 ? 0 : turningPID.calculate(diff, 0);
        turningMotorValue = turningMotorValue > 1 ? 1 : turningMotorValue;
        turningMotorValue = turningMotorValue < -1 ? -1 : turningMotorValue;
        // System.out.println(String.format("DESIRED ANGLE: %f", currentState.angle.getDegrees() - this.turningOffset.getDegrees()));
        // System.out.println(String.format("SPEED SET: %f", turningMotorValue));
        turningMotor.set(turningMotorValue);
    }

    public void gogogo() {
        turningMotor.set(.1);
    }



}
