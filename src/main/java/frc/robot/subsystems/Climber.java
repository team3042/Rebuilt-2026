// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  SparkMax climberMotor;
  DigitalInput climberLimitSwitch;

  public Climber() {

    climberMotor = new SparkMax(Constants.MotorIDs.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberLimitSwitch = new DigitalInput(Constants.DigitalIO.CLIMBER_LIMIT_SWITCH_ID);

  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
