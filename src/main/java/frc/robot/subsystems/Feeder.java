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
  private final SparkMax launcherFeederMotor;

  public Feeder() {

    spindexerMotor = new SparkMax(Constants.MotorIDs.SPIDEXER_MOTOR_ID, MotorType.kBrushless);
    launcherFeederMotor = new SparkMax(Constants.MotorIDs.FEEDER_MOTOR_ID, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
