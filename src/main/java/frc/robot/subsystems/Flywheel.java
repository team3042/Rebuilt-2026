// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
 
  private final SparkMax flywheelMotor;

  public Flywheel() {

    flywheelMotor = new SparkMax(Constants.MotorIDs.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    
  }

  public void runFlywheelMotor(double power) {
    power = Math.max(power, -1);
    power = Math.min(power, 1);

    flywheelMotor.set(power);
  }

  public SparkMax getFlywheelMotorPosition() {

    return flywheelMotor;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
