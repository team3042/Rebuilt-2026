// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */

  private final SparkMax spindexerMotor;
  private final SparkMax feederMotor;

  public Feeder() {

    spindexerMotor = new SparkMax(Constants.MotorIDs.SPIDEXER_MOTOR_ID, MotorType.kBrushless);
    feederMotor = new SparkMax(Constants.MotorIDs.FEEDER_MOTOR_ID, MotorType.kBrushless);

  }

  public void setPowerToSpindexer(double power) {

    power = powerAcceptable(power);
    spindexerMotor.set(power);
  
  }

  public void setPowerToFeeder(double power) {

    power = powerAcceptable(power);
    feederMotor.set(power);
  }

  public double getSpindexerMotorPosition() {

    return spindexerMotor.getEncoder().getPosition();

  }

  public double getFeederMotorPosition() {

    return feederMotor.getEncoder().getPosition();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double powerAcceptable(double power) {

    power = Math.max(power, -1);
    power = Math.min(power, 1);

    return power;
  }
}
