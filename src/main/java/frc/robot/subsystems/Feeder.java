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

    spindexerMotor = new SparkMax(Constants.MotorIDs.SPINDEXER_MOTOR_ID, MotorType.kBrushless);
    launcherFeederMotor = new SparkMax(Constants.MotorIDs.FEEDER_MOTOR_ID, MotorType.kBrushless);

  }

  public void powerToSpindexer(double power) {
    power = Math.min(power, 1);
    power = Math.max(power, -1);

    spindexerMotor.set(power);
  }

  public void powerToFeeder(double power) {
    power = Math.min(power, 1);
    power = Math.max(power, -1);

    launcherFeederMotor.set(power);
  }

  public void stopMotors() {
    spindexerMotor.set(0);
    launcherFeederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
