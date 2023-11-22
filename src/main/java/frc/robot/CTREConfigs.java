// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

/** Add your docs here. */
public class CTREConfigs {
    public static CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
      swerveCanCoderConfig = new CANCoderConfiguration();
  
      /* Swerve CANCoder Configuration */
      swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
      swerveCanCoderConfig.sensorDirection = false;
      swerveCanCoderConfig.initializationStrategy =
          SensorInitializationStrategy.BootToAbsolutePosition;
      swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
